import os

from joint_bench.__main__ import DEFAULT_OUT, REPO


def test_repo_is_the_repository_root():
    assert os.path.isfile(os.path.join(REPO, "tools", "joint_bench", "joint_bench",
                                       "__main__.py"))


def test_recordings_default_to_the_root_debug_bench():
    assert DEFAULT_OUT == os.path.join(REPO, "debug", "bench")
