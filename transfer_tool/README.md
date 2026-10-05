# DJI Fly Mission Transfer

A small desktop helper that does exactly one job: take a `.kmz` built in the
Flight Planner and push it into a mission slot on a connected controller.

**Controller support**: **RC 2** works, confirmed against real hardware. The plain **DJI RC**
(the model without "2" or "Pro" in the name - e.g. RM330) **does not work, and this is not
fixable from the app** - see *Controller support* below before spending time troubleshooting
one. **RC Pro** is untested but not known to fail the same way. RC-N1/RC-N2 (phone-based,
no built-in screen) are also untested.

It exists because the web version of the planner cannot reach USB at all. A
browser has no way to talk to the controller, and the controller is not a
USB drive you can drag files onto, so the web app can only hand you a
downloaded `.kmz`. This helper is the other half: it takes that file from
your computer and puts it on the controller.

Plan a mission on the website, download the `.kmz`, then transfer it here.

## For students - using it

1. Download the app for your operating system and open it.
   - **macOS**: the first time, right-click the app and choose **Open**, then
     **Open** again. Double-clicking will refuse with "unidentified developer"
     or "damaged" until you have done this once. See *Signing* below.
   - **Windows**: the first time, click **More info** then **Run anyway**.
2. On the controller, in DJI Fly, create at least one waypoint mission. The
   helper *replaces* an existing mission rather than creating a new one, so
   there has to be one to overwrite. Anything works - fly two points in your
   driveway and save it.
3. Plug the controller into the computer and switch it on. On the
   controller's screen, set the USB connection to **File Transfer** - it
   starts in charge-only mode and will not hand over any files until you do.
4. On a Mac, **quit Preview, Photos, and Image Capture** if any are open.
   macOS only lets one program talk to the controller at a time, and Preview
   alone being open is enough to stop the transfer.
5. In the helper: **Choose .kmz...**, then **Scan controller**, pick a slot,
   then **Transfer to controller**.
6. Open the mission list in DJI Fly. If the mission is not there straight
   away, back out of the list and open it again.

The preview thumbnail next to each slot shows the mission currently in it, so
you can see what you are about to overwrite.

### Controller support

**RC 2 works.** The plain **DJI RC** (no "2" or "Pro" - e.g. model RM330) **does not, and
no setting on the controller or the computer fixes it.** This was investigated thoroughly,
not just tried once:

The controller connects fine and a USB session opens normally, but the folder DJI Fly stores
missions in (`Android/data/dji.go.v5/files/waypoint`) never appears in what the device is
willing to hand over. Confirmed identically across two completely independent, unrelated USB
backends - `libmtp` on Mac, and Windows' own Portable Devices stack - both before and after
enabling every "show hidden storage" setting reachable on the controller's own screen. Since
two implementations that share no code both see the same restricted view, the restriction is
coming from the controller's own Android build, not from anything the computer or this tool
does - there's no driver, library, or setting on the computer side that can retrieve a folder
the device itself won't advertise. Also confirmed present even with the mission folder
genuinely reachable through the controller's own local file browser at the time of the scan -
so the folder existing was never in question, only whether the controller will describe it to
a connected computer, and it won't.

If a scan never finds any slots on a plain DJI RC, this is why - stop troubleshooting USB
modes or background apps and try a different controller instead.

**RC Pro** hasn't been tested, but nothing found during this investigation suggests it would
fail the same way - treat it as untested, not unsupported. RC-N1/RC-N2 (phone-based, no
built-in screen) are also untested.

### If it cannot find the controller (RC 2)

Two things account for nearly every failure, and neither is guessable from
the error, which surfaces as an unhelpful "could not claim interface" from
deep inside libmtp:

- **Quit Preview, Photos, and Image Capture** (macOS). macOS lets exactly one
  program hold the controller, and these grab camera-like devices on their
  own. **Preview being open is enough to block every transfer.** The helper
  closes some Image Capture helper processes for you, but it does *not* quit
  Preview, Photos, or Image Capture themselves - those are real apps with
  windows and possibly unsaved work, so closing them is left to you. When a
  scan fails, the helper checks which of them is running and names it.
- **Set the controller's USB connection to File Transfer.** The RC 2 is an
  Android device and comes up in charge-only mode: it appears over USB, and
  the helper will even identify it as a DJI Controller 2, while still
  refusing all file access. Look for the USB notification on the controller's
  own screen.

Also worth checking:

- The controller is on, unlocked, and plugged in with a *data* cable - some
  charge-only cables carry no data lines at all.
- **Linux**: if the desktop auto-mounted the controller, eject it first.

## For maintainers - building it

```
pip install pyinstaller     # build machine only, never a student's
python build_core.py        # sync the transfer code out of app.py
python build_installer.py   # -> dist/
```

There is no cross-compiling. Build the Mac app on a Mac and the Windows exe
on Windows, once per platform you hand out. On macOS, build on the oldest
version you intend to support.

Output differs by platform, deliberately:

- **Windows** gets a single `.exe` - the friendliest thing to hand someone.
- **macOS** gets a `.app` bundle, which the build then **zips for you** - hand
  out the `.zip`, never the bare `.app`. A `.app` is a directory, so sending
  it as a bare folder loses the executable bit through most chat and mail
  clients and the app then won't open. Onefile mode is deliberately not used
  on macOS: it unpacks itself to a temp directory on every launch, which is
  slower and is exactly the pattern Gatekeeper treats as suspicious on a
  downloaded app. PyInstaller deprecated that combination and makes it an
  error in v7.

The current Mac build is ~15 MB zipped.

`build_installer.py` refuses to build if anything is off - PyInstaller
missing, libmtp missing, or the transfer code out of sync - rather than
producing a bundle that fails later on someone else's machine.

### How this relates to `app.py`

`dji_transfer_core.py` is **generated** - it is app.py's MTP/WPD layer and its
two transfer functions, copied out verbatim by `build_core.py`. Do not edit
it by hand.

It is a copy rather than a shared import on purpose. app.py deliberately
keeps that bridge inlined (it says why in its own comments), and it is
deployed to a live class, so it is not something to restructure underneath
its users for the convenience of this tool.

A copy is only safe if drift is detectable, so:

```
python build_core.py --check    # non-zero exit if it no longer matches app.py
```

Run that in CI, or before any build. If you fix a controller bug, fix it in
`app.py` and re-run `build_core.py` - otherwise the planner and the helper
quietly disagree about how to talk to a controller.

### What is in the bundle

| Platform | USB backend | Bundled |
|---|---|---|
| macOS / Linux | libmtp via ctypes | libmtp + libusb, copied in and repointed by `rthook_libmtp.py` |
| Windows | Windows Portable Devices (COM) | nothing - WPD ships with Windows |

Windows cannot use libmtp without replacing the controller's driver with
WinUSB, which would break DJI Fly's own access to it - hence the separate
WPD backend, inherited from app.py.

Thumbnails need Pillow. Without it the app still scans and transfers, it just
lists slots without previews.

### Signing

Builds are unsigned, which is why students need the right-click → Open dance
on macOS and "Run anyway" on Windows. Removing that friction means paying for
certificates: an Apple Developer account (~$99/yr) plus notarisation for
macOS, and an Authenticode certificate for Windows. Worth it for a large
class, hard to justify for a small one - but tell people about the warning in
advance either way, or the first support question will be "it says the app is
damaged".

## Running from source

```
python dji_transfer_app.py
```

Needs `libmtp` on macOS/Linux (`brew install libmtp`), or `comtypes` on
Windows. Pillow is optional, for the preview thumbnails.
