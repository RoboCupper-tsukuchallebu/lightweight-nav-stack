"""CI の colcon は pytest の失敗詳細を出さないので、失敗の要約を終了時に stderr へ出す。"""

import atexit
import sys

failure_reports = []


def pytest_runtest_logreport(report):
    if report.failed:
        failure_reports.append(f"FAILED {report.nodeid} ({report.when})\n{report.longreprtext}")


@atexit.register
def print_failure_reports():
    for text in failure_reports:
        print(text[-3000:], file=sys.__stderr__, flush=True)
