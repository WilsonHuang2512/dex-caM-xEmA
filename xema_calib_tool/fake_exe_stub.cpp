// fake_exe_stub.cpp
//
// A controllable stand-in for calibration.exe / open_cam3d.exe. Build this once, then drop
// TWO renamed copies (fake_exe_stub.exe -> calibration.exe, fake_exe_stub.exe ->
// open_cam3d.exe) into the test binary's own directory. runExeBlocking always resolves these
// names relative to QCoreApplication::applicationDirPath(), so a test binary sitting next to
// these fakes will call them instead of the real vendor exes -- no source changes needed in
// XemaCameraWindow.cpp to make this substitution happen.
//
// Controlled entirely via environment variables (set by the test process before spawning the
// window under test, or via qputenv() in test setup) so each test case can dictate different
// exe behavior without recompiling the stub:
//
//   FAKE_EXE_DELAY_MS       -- how long to sleep before doing anything else (default 0).
//                              Use this to test the 60s/600s timeouts in runExeBlocking, or to
//                              create a reliable window for a second click to race against.
//   FAKE_EXE_EXIT_CODE      -- exit code to return (default 0). Calibrate/Correct/write-params
//                              all treat exit code as UNRELIABLE by design (see the "exit code
//                              lies" comment in calibrateThreadFunc) -- set this to a nonzero
//                              value to confirm the tool still reports success correctly when
//                              the real evidence (output file) says success anyway.
//   FAKE_EXE_WRITE_FILE     -- if set, path to write a small placeholder file to after the
//                              delay (simulates calibration.exe writing param.txt, or
//                              open_cam3d.exe writing phase36.bmp / a captured pose). If unset,
//                              no file is written -- use this to test the failure path where
//                              the expected output never appears.
//   FAKE_EXE_STDOUT_FILE    -- if set, path to a text file whose contents get echoed to stdout
//                              verbatim (use this to feed a real captured calibration.exe
//                              transcript through logCalibExeOutput's parsing, so its
//                              board-count/reprojection-error summary logic gets exercised
//                              against real-looking text without needing calibration.exe
//                              itself).
//
// This intentionally ignores its actual argv -- the tests assert on the ARGS BUILT before the
// call (via the proposed buildCalibrateArgs()/buildCorrectArgs() free functions), not on what
// this stub receives, so the stub's only job is to behave like a slow/fast, succeeding/failing
// exe, not to validate its own command line.

#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>

static int getEnvInt(const char* name, int fallback)
{
	const char* v = std::getenv(name);
	if (!v || !*v) return fallback;
	return std::atoi(v);
}

static std::string getEnvStr(const char* name)
{
	const char* v = std::getenv(name);
	return v ? std::string(v) : std::string();
}

int main(int /*argc*/, char** /*argv*/)
{
	int delay_ms = getEnvInt("FAKE_EXE_DELAY_MS", 0);
	int exit_code = getEnvInt("FAKE_EXE_EXIT_CODE", 0);
	std::string write_file = getEnvStr("FAKE_EXE_WRITE_FILE");
	std::string stdout_file = getEnvStr("FAKE_EXE_STDOUT_FILE");

	if (delay_ms > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
	}

	if (!stdout_file.empty())
	{
		std::ifstream in(stdout_file, std::ios::binary);
		if (in)
		{
			std::fputs(std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()).c_str(), stdout);
		}
	}
	else
	{
		std::printf("fake_exe_stub: ran with delay=%dms, will exit %d\n", delay_ms, exit_code);
	}

	if (!write_file.empty())
	{
		std::ofstream out(write_file, std::ios::binary);
		out << "fake output from fake_exe_stub\n";
	}

	return exit_code;
}
