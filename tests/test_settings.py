"""The receiver's live settings: what an operator may set, and what is refused."""

from __future__ import annotations

import pytest

from keelson_sil_gnss.settings import Settings, SettingsError, merge


def test_a_request_changes_only_what_it_names():
    settings = merge(Settings(grade="survey"), {"jamming": 0.4})
    assert settings.grade == "survey"
    assert settings.faults.jamming == 0.4
    assert settings.faults.multipath == 0.0


def test_the_grade_can_be_switched_at_run_time():
    assert merge(Settings(), {"grade": "professional"}).grade == "professional"


@pytest.mark.parametrize(
    "request_",
    [
        {"grade": "military"},
        {"jamming": 1.5},
        {"jamming": "a lot"},
        {"jamming": True},
        {"corrections": "no"},
        {"jaming": 0.3},
        {"spoofing": {"offset_m": -5}},
        {"spoofing": {"offset_m": 5, "heading": 3}},
        {"spoofing": 12},
        "jamming",
    ],
)
def test_what_is_not_a_setting_is_refused_rather_than_ignored(request_):
    with pytest.raises(SettingsError):
        merge(Settings(), request_)


def test_spoofing_starts_at_the_instant_given_and_stops_on_null():
    on = merge(Settings(), {"spoofing": {"offset_m": 50, "bearing_deg": 450, "drift_mps": 1}}, 123)
    assert on.faults.spoofing
    assert on.faults.spoof_bearing_deg == 90.0
    assert on.faults.spoof_since_ns == 123
    off = merge(on, {"spoofing": None})
    assert not off.faults.spoofing and off.faults.spoof_since_ns is None


def test_clear_takes_every_fault_off_and_keeps_the_grade():
    busy = merge(
        Settings(grade="survey"),
        {"jamming": 0.5, "sky_mask": 0.3, "receiver_on": False, "lock_duty": 0.2,
         "corrections": False, "spoofing": {"offset_m": 10}},
    )
    clean = merge(busy, {"clear": True})
    assert clean == Settings(grade="survey")


def test_the_config_says_what_is_set_and_which_grades_there_are():
    config = merge(Settings(), {"multipath": 0.25}).as_config()
    assert config["multipath"] == 0.25
    assert config["spoofing"] is None
    assert set(config["grades"]) == {"recreational", "professional", "survey"}
