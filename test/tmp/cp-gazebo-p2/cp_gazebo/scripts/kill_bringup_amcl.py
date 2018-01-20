#!/usr/bin/python2.7
import os
import subprocess
import sys
from subprocess import Popen, PIPE

def error(msg):
    print("error: {}".format(msg))
    sys.exit(1)

class NonZeroExitException(Exception):
    def __init__(self,retcode,stderr):
        super(self.__class__,self).__init__()
        self.exceptioncode = retcode
        self.stderr = stderr

class cmdTimeOut(Exception):
    pass

def kill_amcl():
    try:
        bash_exec("rosnode kill /amcl")
    except NonZeroExitException as e:
        error("failed to kill /amcl node")
    except cmdTimeOut as e:
        error("timeoout")

def bringup_amcl():
    try:
        bash_exec("rosrun amcl amcl")
    except NonZeroExitException as e:
        error("failed to bringup /amcl node")
    except cmdTimeOut as e:
        error("timeoout")

def bash_exec(cmd,timeout=10):
        try:
            pr = Popen(cmd, shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE, preexec_fn=os.setsid)
            stdout, stderr = pr.communicate(timeout=timeout)
            if pr.returncode != 0:
                raise NonZeroExitException(pr.returncode, stderr)
            return stdout

        except subprocess.TimeoutExpired:
            raise cmdTimeOut()
        finally:
            os.killpg(pr.pid, 9)

def main():
    kill_amcl()
    bringup_amcl()


if __name__=="__main__":
    main()