"""
DJI Fly Mission Transfer - a small desktop helper.

Does one job: take a .kmz built in the Flight Planner (the web version
included, which can't reach USB at all) and push it into a mission slot on a
connected RC 2. Nothing else from the planner is here - no map, no mission
editing - so this can be frozen into a single download that needs neither
Python nor any libraries installed.

Run from source:   python dji_transfer_app.py
Build a binary:    python build_installer.py       (see that script's notes)

All controller work happens on a background thread. The MTP calls block for
seconds at a time - a scan walks several device folders, and a transfer
deliberately sleeps 3.5s mid-way for Android's MediaStore to catch up - and
doing that on Tk's main thread would freeze the window and earn an
"application not responding" badge from the OS partway through every push.
Workers therefore talk back through a Queue that the UI drains on a timer;
nothing but the main thread ever touches a widget.
"""
import os
import queue
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import dji_transfer_core as core

# Thumbnails are JPEGs and Tk can't decode those on its own, so previews are
# a Pillow-shaped luxury rather than a requirement: without it the tool still
# scans and transfers, it just lists slots by UUID instead of showing what is
# currently sitting in each one.
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# Somewhere disposable and always writable. Not a folder next to the program:
# a frozen build can sit inside a read-only .app bundle or Program Files.
CACHE_DIR = os.path.join(tempfile.gettempdir(), "dji_fly_transfer_cache")

# The preview is the only thing that actually identifies a mission on the
# controller - the slot number is just a handle for pointing at one - so it
# gets the room, big enough to recognise a flight path at a glance rather
# than merely proving a thumbnail exists.
THUMB_HEIGHT = 135
# Previews are 16:9, so height alone would let a wide one spill past its
# column and sit underneath the neighbouring text. Cap the width too and give
# the column a little more room than the cap.
THUMB_MAX_WIDTH = 240
THUMB_COLUMN_WIDTH = THUMB_MAX_WIDTH + 16
# Enough for the slot number and nothing more; the rest goes to the preview.
SLOT_COLUMN_WIDTH = 150
WINDOW_TITLE = "DJI Fly Mission Transfer"


class TransferApp:
    def __init__(self, root):
        self.root = root
        self.kmz_path = None
        self.folder = None
        self.nests = {}
        self.events = queue.Queue()
        self.busy = False
        # Tk drops any image that isn't still referenced from Python, so the
        # PhotoImages have to outlive the loop that builds the rows.
        self._thumbnails = []

        root.title(WINDOW_TITLE)
        root.minsize(620, 700)
        self._build_ui()
        self._check_backend()
        self.root.after(100, self._drain_events)

    # ---------------------------------------------------------------- UI

    def _build_ui(self):
        pad = {"padx": 12, "pady": 6}
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)

        # Step 1 - the mission file
        step1 = ttk.LabelFrame(outer, text="1.  Mission file")
        step1.pack(fill="x", **pad)

        # Two ways in, because missions arrive two ways: one-off downloads
        # (pick the file) and a folder the planner has been saving into all
        # session (pick the folder once, then work through its missions
        # without re-navigating a file dialog for each one).
        row = ttk.Frame(step1)
        row.pack(fill="x", padx=10, pady=(8, 2))
        self.file_label = ttk.Label(row, text="No mission chosen", foreground="grey")
        self.file_label.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Choose .kmz...", command=self.choose_file).pack(side="right")

        folder_row = ttk.Frame(step1)
        folder_row.pack(fill="x", padx=10, pady=(2, 2))
        self.folder_label = ttk.Label(
            folder_row, text="or choose a folder of missions", foreground="grey",
        )
        self.folder_label.pack(side="left", fill="x", expand=True)
        ttk.Button(folder_row, text="Choose folder...", command=self.choose_folder).pack(side="right")

        pick_row = ttk.Frame(step1)
        pick_row.pack(fill="x", padx=10, pady=(2, 8))
        self.folder_pick = ttk.Combobox(pick_row, state="disabled", values=())
        self.folder_pick.pack(fill="x")
        self.folder_pick.bind("<<ComboboxSelected>>", self._folder_file_chosen)

        # Step 3 and the status line are packed against the bottom BEFORE the
        # slot list claims its space. pack() hands out space in call order, so
        # with the expanding list added first the button gets pushed off the
        # window entirely once the list has any real height to it.
        self.status = ttk.Label(outer, text="Ready.", foreground="grey", anchor="w")
        self.status.pack(side="bottom", fill="x", padx=12, pady=(0, 10))
        self.transfer_button = ttk.Button(
            outer, text="Transfer to controller", command=self.transfer, state="disabled",
        )
        self.transfer_button.pack(side="bottom", fill="x", **pad)

        # Step 2 - the controller
        step2 = ttk.LabelFrame(outer, text="2.  Mission slot on the controller")
        step2.pack(fill="both", expand=True, **pad)

        bar = ttk.Frame(step2)
        bar.pack(fill="x", padx=10, pady=(8, 2))
        self.scan_button = ttk.Button(bar, text="Scan controller", command=self.scan)
        self.scan_button.pack(side="left")

        # Said up front rather than only in the error, because the error
        # arrives after the confusion: macOS lets exactly one program hold the
        # controller, and the resulting failure names none of them.
        ttk.Label(
            step2,
            text=(
                "Before scanning: quit Preview, Photos, and Image Capture. macOS lets only\n"
                "one program talk to the controller, and these grab it automatically.\n"
                "Controller Support: RC 2. RC does not work. RC Pro untested."
            ),
            foreground="grey", justify="left",
        ).pack(anchor="w", padx=12, pady=(0, 6))

        # Row height has to be set on the style, not the widget, or the
        # thumbnails are clipped to the default single-line height.
        style = ttk.Style()
        style.configure("Nests.Treeview", rowheight=THUMB_HEIGHT + 8)

        holder = ttk.Frame(step2)
        holder.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        self.tree = ttk.Treeview(
            holder, style="Nests.Treeview", columns=("slot",),
            show="tree headings" if PIL_AVAILABLE else "headings", selectmode="browse",
        )
        self.tree.heading("slot", text="Slot")
        self.tree.column("#0", width=THUMB_COLUMN_WIDTH, stretch=False)
        self.tree.column("slot", anchor="w", width=SLOT_COLUMN_WIDTH)
        scroll = ttk.Scrollbar(holder, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._refresh_transfer_button())

    def _check_backend(self):
        """Say up front if this machine can't talk to a controller at all."""
        if core.get_mtp_session_class() is not None:
            return
        self.scan_button.state(["disabled"])
        if sys.platform == "win32":
            detail = "The Windows Portable Devices backend didn't load."
        else:
            detail = ("libmtp isn't installed. On a Mac with Homebrew:\n\n"
                      "    brew install libmtp")
        self._set_status("No USB backend available - transfers are disabled.", error=True)
        messagebox.showerror(
            WINDOW_TITLE,
            "This computer can't talk to the controller.\n\n" + detail,
        )

    @staticmethod
    def _running_hijackers():
        """
        macOS apps currently running that are known to seize the controller.

        The failure they cause is a libusb "could not claim interface" deep in
        libmtp, which says nothing about which program is at fault - so rather
        than make someone quit apps one at a time until it works, check and
        name the actual offender. Best-effort only: an empty list means
        nothing was found, not that nothing is holding the device.
        """
        if sys.platform != "darwin":
            return []
        found = []
        for process, friendly in (
            ("Preview", "Preview"),
            ("Photos", "Photos"),
            ("Image Capture", "Image Capture"),
            ("AndroidFileTransfer", "Android File Transfer"),
            ("OpenMTP", "OpenMTP"),
        ):
            try:
                hit = subprocess.run(["pgrep", "-x", process], capture_output=True, timeout=2)
            except (OSError, subprocess.SubprocessError):
                continue
            if hit.returncode == 0:
                found.append(friendly)
        return found

    def _set_status(self, text, error=False):
        self.status.configure(text=text, foreground="#b00020" if error else "grey")

    def _hijacker_hint(self):
        """Trailing line naming whichever known offender is running, if any."""
        running = self._running_hijackers()
        if not running:
            return ""
        return (
            "\n\n" + " and ".join(running) + " is currently open. macOS lets only one "
            "program talk to the controller at a time - quit it and scan again."
            if len(running) == 1 else
            "\n\n" + ", ".join(running) + " are currently open. macOS lets only one "
            "program talk to the controller at a time - quit them and scan again."
        )

    def _set_busy(self, busy, status=None):
        self.busy = busy
        state = "disabled" if busy else "!disabled"
        self.scan_button.state([state])
        if status:
            self._set_status(status)
        self._refresh_transfer_button()

    def _refresh_transfer_button(self):
        ready = bool(self.kmz_path) and bool(self.tree.selection()) and not self.busy
        self.transfer_button.state(["!disabled" if ready else "disabled"])

    # ------------------------------------------------------------ actions

    def choose_file(self):
        path = filedialog.askopenfilename(
            title="Choose a mission",
            filetypes=[("DJI mission", "*.kmz"), ("All files", "*.*")],
        )
        if not path:
            return
        self._set_mission(path)

    def choose_folder(self):
        folder = filedialog.askdirectory(title="Choose a folder of missions")
        if not folder:
            return
        try:
            missions = sorted(
                name for name in os.listdir(folder) if name.lower().endswith(".kmz")
            )
        except OSError as exc:
            messagebox.showerror(WINDOW_TITLE, f"Could not read that folder:\n\n{exc}")
            return

        self.folder = folder
        self.folder_label.configure(text=folder, foreground="black")
        self.folder_pick.configure(values=missions)
        self.folder_pick.set("")
        if not missions:
            # Leave any already-chosen mission alone - an empty folder is a
            # wrong turn, not an instruction to forget the current file.
            self.folder_pick.configure(state="disabled")
            messagebox.showwarning(
                WINDOW_TITLE, f"No .kmz missions in that folder:\n\n{folder}",
            )
            return
        self.folder_pick.configure(state="readonly")
        self._set_status(f"{len(missions)} mission(s) in that folder - pick one.")

    def _folder_file_chosen(self, _event=None):
        name = self.folder_pick.get()
        if not name or not self.folder:
            return
        self._set_mission(os.path.join(self.folder, name))

    def _set_mission(self, path):
        self.kmz_path = path
        self.file_label.configure(text=os.path.basename(path), foreground="black")
        self._refresh_transfer_button()

    def scan(self):
        self._clear_nests()
        self._set_busy(True, "Scanning controller - this takes a few seconds...")
        self._run_bg(
            lambda: core.fetch_controller_nests_and_previews(
                cache_dir=CACHE_DIR, pull_thumbnails=PIL_AVAILABLE,
            ),
            self._scan_done,
        )

    def transfer(self):
        uuid = self._selected_uuid()
        if not uuid or not self.kmz_path:
            return
        # The push purges the slot before writing, so it is genuinely
        # destructive to whatever mission is sitting there - worth one
        # confirmation rather than an undo that doesn't exist.
        if not messagebox.askyesno(
            WINDOW_TITLE,
            f"Replace the mission in this slot with:\n\n{os.path.basename(self.kmz_path)}\n\n"
            "Whatever is in the slot now will be overwritten.",
        ):
            return
        self._set_busy(True, "Transferring - do not unplug the controller...")
        self._run_bg(lambda: core.push_mission_to_nest(self.kmz_path, uuid), self._transfer_done)

    # ------------------------------------------------- background plumbing

    def _run_bg(self, work, on_done):
        """Run `work()` off the main thread; deliver its result to `on_done`."""
        def runner():
            try:
                self.events.put((on_done, work(), None))
            except Exception as exc:  # noqa: BLE001 - surfaced in the UI below
                core.logger.exception("background task failed")
                self.events.put((on_done, None, exc))

        threading.Thread(target=runner, daemon=True).start()

    def _drain_events(self):
        """Deliver finished background work on the main thread."""
        try:
            while True:
                on_done, result, error = self.events.get_nowait()
                on_done(result, error)
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    # ------------------------------------------------------------ results

    def _scan_done(self, result, error):
        self._set_busy(False)
        if error is not None:
            self._set_status(f"Scan failed: {error}", error=True)
            messagebox.showerror(WINDOW_TITLE, f"Scan failed:\n\n{error}")
            return

        nests, _preview_id, message = result
        if message:
            self._set_status(message, error=True)
            messagebox.showerror(WINDOW_TITLE, message + self._hijacker_hint())
            return
        if not nests:
            self._set_status("No controller found, or no mission slots on it.", error=True)
            messagebox.showwarning(
                WINDOW_TITLE,
                "No mission slots found.\n\n"
                "Check that the controller is plugged in and switched on, and that you "
                "have created at least one mission in DJI Fly for this tool to overwrite."
                + self._hijacker_hint(),
            )
            return

        self.nests = nests
        self._populate_nests(nests)
        self._set_status(f"Found {len(nests)} mission slot(s). Choose one, then transfer.")

    def _transfer_done(self, result, error):
        self._set_busy(False)
        if error is not None:
            self._set_status(f"Transfer failed: {error}", error=True)
            messagebox.showerror(WINDOW_TITLE, f"Transfer failed:\n\n{error}")
            return

        ok, message = result
        if ok:
            self._set_status("Transfer complete.")
            messagebox.showinfo(
                WINDOW_TITLE,
                "Mission transferred.\n\nOpen DJI Fly on the controller to find it in "
                "the mission list. If it doesn't appear straight away, back out of "
                "the mission list and open it again.",
            )
        else:
            self._set_status(f"Transfer failed: {message}", error=True)
            messagebox.showerror(WINDOW_TITLE, f"Transfer failed:\n\n{message}")

    # -------------------------------------------------------- nest list

    def _clear_nests(self):
        self.tree.delete(*self.tree.get_children())
        self._thumbnails.clear()
        self.nests = {}
        self._refresh_transfer_button()

    def _populate_nests(self, nests):
        # The preview is what actually tells someone which mission they are
        # about to overwrite, so the text column carries the handle for it -
        # a slot number - rather than the raw UUID, which is unreadable and
        # identical-looking across slots at a glance. The UUID is still the
        # row's iid, so it is what gets passed to the transfer.
        for number, uuid in enumerate(sorted(nests), start=1):
            image = self._load_thumbnail(uuid)
            if image is None:
                label = f"Slot {number}   (no preview available)"
            else:
                self._thumbnails.append(image)
                label = f"Slot {number}"
            self.tree.insert("", "end", iid=uuid, image=image or "", values=(label,))

    def _load_thumbnail(self, uuid):
        if not PIL_AVAILABLE:
            return None
        path = os.path.join(CACHE_DIR, f"{uuid}.jpg")
        if not os.path.exists(path):
            return None
        try:
            with Image.open(path) as img:
                img.load()
                ratio = min(THUMB_HEIGHT / float(img.height), THUMB_MAX_WIDTH / float(img.width))
                img = img.resize((max(1, int(img.width * ratio)), max(1, int(img.height * ratio))))
                return ImageTk.PhotoImage(img)
        except Exception:
            core.logger.exception("could not load preview for %s", uuid)
            return None

    def _selected_uuid(self):
        selection = self.tree.selection()
        return selection[0] if selection else None


def main():
    root = tk.Tk()
    TransferApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
