# xema_calib_tool — stability test plan (draft 1)

Two tiers, because this app is a Windows GUI wired to real hardware and external
.exe processes — most of it can't be unit-tested in the traditional sense, but the
logic *underneath* the Qt/thread plumbing can be, and that's where bugs like the
`busy_` lockout hide.

## Tier 1 — pure logic (automatable now)

These don't touch the camera, a QProcess, or the filesystem in a way that needs
mocking — they're string/path/struct transforms. Pulling them out into free
functions (see `xema_logic_test.cpp` below for a starter) makes them testable
with plain QtTest, no camera or exe required.

| Function | Edge cases worth a test case |
|---|---|
| `identityFolderName()` | empty identity field → `"default"`; MAC with colons (`AA:BB:CC:DD:EE:FF`); MAC with dashes; identity with embedded spaces; identity that's *already* underscore-separated (idempotent); Chinese/unicode characters in identity (not sanitized today — worth deciding on purpose) |
| `currentIdentityFolder()` | save-path field empty → falls back to exe dir; save-path with trailing slash (double-slash risk: `path/` + `/` + `name`); save-path using forward slashes vs backslashes on Windows; save-path with a trailing `\` from a native file-picker |
| calibrate/correct arg builders (currently inline in `calibrateThreadFunc`/`correctThreadFunc` — worth extracting, see below) | mono vs color → `patterns` vs `patterns-c`; each of the 5 board spacings; both projector versions (3010/4710); an *unknown* projector version (0, or a stale value from before ever connecting) — does it silently send `--version 0`? |
| `logMissingBoardPoses` diffing | 0 poses on disk; pose folder exists but is empty; a pose with `_board.bmp` but no `_draw.bmp` (not found); a pose with `_draw.bmp` but *no* matching `_board.bmp` (shouldn't happen, but if it does — does it crash or just get skipped silently?) |
| `saveConfig`/`loadConfig` round-trip | first-ever run (no config.json) → does it fall back to sane defaults instead of crashing; a config.json missing the newer `calib_mode` key (old config from before this feature existed) → must default to `"color"`, not crash; a hand-edited config.json with a garbage value for `calib_mode` (e.g. `"banana"`) or `board_spacing_mm` (e.g. `999`) — does the fallback `default:` case in the switch actually catch it, or does an invalid *enum* value slip through uninitialized? |

## Tier 2 — click-sequence / concurrency (manual, but scriptable as a checklist)

This is where the `busy_` bug lived — a flag set on click, never reset on
completion. The fix pattern is now consistent (every guard flag has a matching
reset in its `on...Finished` handler), but the way to *catch the next one* is a
deliberate click-sequence pass, since nothing forces symmetry at compile time.

**Method:** for every button that starts a background thread (Connect, Capture
toggle, Capture-for-calib, Calibrate, Write-params, Correct, Apply-params,
Disconnect), run this matrix:

1. Click it once, let it finish normally → confirm the button (and everything
   else it disables) is clickable again afterward.
2. Click it, then **immediately** click it again before it finishes → confirm
   you get the "already in progress" warning, not a second thread launched.
3. Click it, then click every *other* connection-exclusive button while it's
   still running → confirm each gives the "another operation in progress"
   warning rather than launching a second exe/DfConnect call concurrently.
4. Force it to fail (wrong IP, camera powered off, exe missing/renamed) →
   confirm the button re-enables afterward instead of staying locked.
5. Force it to take unusually long (or literally unplug the camera mid-run) →
   confirm the timeout paths (`kCaptureStopTimeoutMs`, the 60s/600s exe
   timeouts) actually fire and leave the UI in a recoverable state rather than
   hanging forever.

Specific sequences to prioritize (based on real code paths already touching
`connected_`/`capturing_` from multiple threads):

- Start continuous capture → click Capture-for-calib → does it correctly stop
  continuous capture, run its own capture, then **resume** continuous capture
  afterward? (this exact resume logic is duplicated across
  `captureForCalibThreadFunc`, `writeParamsThreadFunc`, and `correctThreadFunc`
  — a regression in one wouldn't necessarily show up in the others)
- Click Correct while *not connected* (should skip disconnect/reconnect
  entirely, per the recent fix) → then click Correct again while *connected* →
  confirm both code paths still leave `connected_` in the correct final state.
- Click Disconnect while Capture-for-calib/Write-params/Correct is mid-run
  (their own internal `DfDisconnect` racing against a user-initiated one).
- Kill the camera's network connection (unplug cable) mid-`captureLoopThreadFunc`
  → does the next Connect attempt behave correctly, or is there stale state
  left over from the interrupted loop?

## Tier 3 — filesystem / exe integration (manual, needs the real hardware)

- `calibration.exe`/`open_cam3d.exe` missing or renamed → confirm the
  STATUS_DLL_NOT_FOUND-style silent failure is caught and reported, not just a
  hang or a misleading "success."
- Exit code vs. file-existence mismatch (already specifically handled for
  Calibrate/Correct's stale-file-delete-then-check pattern) — worth a
  regression test: pre-create a fake `param.txt` with junk content, run
  Calibrate/Correct against a pose folder that will *fail* board detection,
  confirm the stale file really does get deleted and the run is correctly
  reported as failed rather than silently "succeeding" on the leftover file.
- Save-path pointing at a read-only or nonexistent drive.
- Identity folder with 0 poses vs. 1 pose vs. exactly the calibrate/correct
  minimum (per your own notes: ~25 poses for full calibrate, ~5 for correct) —
  does the tool warn *before* running the exe, or just let it fail?
- Two instances of the tool pointed at the same camera IP simultaneously.

## Suggested next step

I'd start with Tier 1 since it's the only part that's actually automatable
today, and it directly covers the config/path logic your correct-mode work
just touched. See the starter test file below — it assumes a small refactor
(pulling the arg-builder logic out of the thread functions into free
functions) which I'd want your go-ahead on before touching
`XemaCameraWindow.cpp` again, since it changes function signatures beyond just
adding to them.

## Tier 2 automation (added: yes, this is doable)

Two depths, in order of how much infrastructure they need:

**A) Click-race tests — zero mocking needed.** Every guard flag
(`calibrating_`, `writing_params_`, `correcting_`, `calib_capturing_`,
`busy_`) is set `true` synchronously on the GUI thread *before* the
background `std::thread` is spawned. That means calling the same
`onXClicked()` slot twice in a row with no event-loop spin in between is
100%-deterministic, not a timing gamble — the second call always hits the
guard, regardless of whether the first background thread has even started.
`QMetaObject::invokeMethod(&w, "onCalibrateClicked")` calls a private slot
directly without needing a `friend` declaration, so this needs almost no
changes to the real class — see `tst_xema_click_races.cpp`.

Two things this tier does need, both trivial:
- Six one-line test-only accessors on `XemaCameraWindow`, guarded by
  `#ifdef XEMA_TEST_HOOKS` so they compile to nothing in the real build (see
  the top of the test file for the exact lines).
- A deliberately unreachable test IP: `192.0.2.10` (RFC 5737 TEST-NET-1,
  guaranteed non-routable). This matters for safety, not just convenience —
  without it, a test that accidentally runs on a machine with real camera
  network access could send real `DfConnect`/`DfSetParamLedCurrent`/etc.
  calls to real hardware. Never swap this for a real or guessed camera IP in
  automated tests.

This tier directly pins down the exact bug this conversation started with —
`connectFinished_alwaysResetsBusyFlag_evenOnFailure` in the test file is the
literal regression test for the `busy_` lockout.

**B) Full sequencing tests — needs two stand-ins.**

- `fake_exe_stub.cpp`: a controllable replacement for `calibration.exe`/
  `open_cam3d.exe`, driven by environment variables (`FAKE_EXE_DELAY_MS`,
  `FAKE_EXE_EXIT_CODE`, `FAKE_EXE_WRITE_FILE`, `FAKE_EXE_STDOUT_FILE`). Build
  it once, drop two renamed copies into the test binary's own folder (since
  `runExeBlocking` always resolves these names relative to
  `applicationDirPath()`), and every test controls its behavior purely
  through env vars — no recompiling per test case. This alone (no SDK mock
  needed) unlocks deterministic testing of Calibrate's logic end-to-end,
  since Calibrate never touches the camera SDK at all — only Correct/
  Write-params/Capture-for-calib do.
- A fake SDK DLL exporting the same symbols (`DfConnect`, `DfDisconnect`,
  `DfSetCaptureEngine`, `DfSetParamLedCurrent`, `DfSetParamCameraExposure`,
  `DfSetParamCameraGain`, etc. — confirmed real signatures from
  `XEMA-master/sdk/DF8.h`/`open_cam3d.h`), also env-var-controlled, linked in
  place of the real vendor `.lib` for a test-only build target. This is the
  piece that would let Correct/Write-params/Capture-for-calib's full
  disconnect→run→reconnect→resume sequencing be tested deterministically
  (including the "was it actually connected before Correct started" branch we
  just added) without any real camera. Bigger lift than (A) — worth doing
  once (A) is running and has already earned its keep, rather than starting
  here.

