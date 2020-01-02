import os
import subprocess
from shlex import split

API_VERSION = "0.1"

class HelperFunctions(object):
    """
    Helper functions that are hardware independent
    """
    class ScriptException(Exception):
        def __init__(self, returncode, stdout, stderr, script):
            self.returncode = returncode
            self.stdout = stdout
            self.stderr = stderr
            Exception.__init__(self, "ScriptException error.")

        def __str__(self):
            ret_str = ', '.join(['{key}={value}'.format(key=key, value=self.__dict__.get(key))
                                 for key in self.__dict__])
            return ret_str

    def execute_cmd_output_string(self, cmd, enable_shell=False):
        """
        Execute a command and return its output as a string.

        :param cmd: abs path of the command with arguments
        :param enable_shell : force the cmd to be run as shell script
        :return: a string.
        """
        try:
            result = subprocess.check_output(split(cmd), stderr=subprocess.STDOUT, shell=enable_shell)

        except subprocess.CalledProcessError as e:
            s = """While executing '{}' something went wrong.
                Return code == '{}'
                Return output:\n'{}'
                """.format(cmd, e.returncode, e.output, shell=enable_shell)
            raise AssertionError(s)

        return result.strip().decode("utf-8").split("\n")

    def execute_cmd_return_code(self, cmd, no_output=True, enable_shell=False):
        """
        Execute a command and return its exit code as a integer.

        :param cmd: abs path of the command with arguments.
        :param no_output: Output is printed or not.
        :param enable_shell : force the cmd to be run as shell script
        :return: a string.
        """
        if no_output is True:
            null = open(os.devnull, 'w')
            return subprocess.call(cmd.split(), stdout=null, stderr=null, shell=enable_shell)
        else:
            return subprocess.call(cmd.split())

    
    def shell_exec_s(self, cmd, debug=True):
        resp = self.execute_cmd_output_string(cmd)
        print('[shell_exec_s:{}] {}'.format(resp, cmd))
        return resp
        # return self.execute_cmd_output_string(cmd)

    def shell_exec_i(self, cmd, debug=True):
        resp =  self.execute_cmd_return_code(cmd)
        print('[shell_exec_i:{}] {}'.format(resp, cmd))
        return resp
        # return self.execute_cmd_return_code(cmd)

    def dir_exists(self, path):
        """
        Check if the directory exists

        :param path: The full path of the directory
        :param ip: The IP if it's a remote target. Default is None
        :return: True if path exists, else Fault
        """
        cmd = '[ -d "%s" ]' % path
        return True if not self.shell_exec_i(cmd) else False

    def file_exists(self, file_path):
        """
        Check if the file exists

        :param file_path: The full path of the file
        :param ip: The IP if it's a remote target. Default is None
        :return: True if file exists, else Fault
        """
        cmd = '[ -f "%s" ]' % file_path
        return True if not self.shell_exec_i(cmd) else False