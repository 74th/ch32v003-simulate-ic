from invoke.context import Context
from invoke.tasks import task

DEVICE_ID = "/dev/sda1"
DEVICE_DIR = "/media/circuitpython"


@task
def mount(c: Context) -> None:
    c.run(f"sudo mount -o uid=$(id -u),gid=$(id -g) UUID=0076-F684 {DEVICE_DIR}")


@task
def umount(c: Context) -> None:
    c.run(f"sudo umount {DEVICE_DIR}")


@task
def upload_mcp23017(c: Context) -> None:
    c.run(f"cp ./src/tester_rp2040/mcp23017.py {DEVICE_DIR}/code.py")
    c.run("sync")


@task
def upload_i2c_test(c: Context) -> None:
    c.run(f"cp ./src/tester_rp2040/i2c_test.py {DEVICE_DIR}/code.py")
    c.run("sync")
