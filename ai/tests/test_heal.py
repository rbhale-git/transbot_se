"""Tests for ai.common.heal — the SSH rosbridge-restart cure.

The ssh subprocess itself is not unit-tested (verified live, like every
connection path in this suite); these tests cover the command shape, the
failure reporting, and the wait-for-port loop with injected fakes.
"""

import types

from ai.common.heal import (
    SERVICE, SSH_USER, heal, host_from_url, restart_command,
)


def proc(returncode=0, stderr="", stdout=""):
    return types.SimpleNamespace(returncode=returncode, stderr=stderr,
                                 stdout=stdout)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


class TestHostFromUrl:
    def test_strips_scheme_and_port(self):
        assert host_from_url("ws://192.168.0.109:9090") == "192.168.0.109"

    def test_no_port(self):
        assert host_from_url("ws://robot.local/") == "robot.local"


class TestRestartCommand:
    def test_non_interactive_ssh_and_sudo(self):
        cmd = restart_command("192.168.0.109")
        assert cmd[0] == "ssh"
        assert "BatchMode=yes" in cmd          # never hang on a password prompt
        assert f"{SSH_USER}@192.168.0.109" in cmd
        assert "sudo" in cmd and "-n" in cmd   # passwordless sudo only
        assert SERVICE in cmd


class TestHeal:
    def test_ssh_failure_reports_stderr(self):
        ok, detail = heal("h", run=lambda *a, **k: proc(255, "Permission denied"),
                          port_open=lambda h, p: True)
        assert ok is False
        assert "Permission denied" in detail

    def test_waits_for_port_to_return(self):
        clock = FakeClock()
        answers = iter([False, False, True])
        ok, detail = heal("h", run=lambda *a, **k: proc(),
                          port_open=lambda h, p: next(answers),
                          sleep=clock.sleep, clock=clock)
        assert ok is True

    def test_times_out_when_port_never_returns(self):
        clock = FakeClock()
        ok, detail = heal("h", timeout_s=10.0, run=lambda *a, **k: proc(),
                          port_open=lambda h, p: False,
                          sleep=clock.sleep, clock=clock)
        assert ok is False
        assert "not back" in detail
