import pexpect
import docker
from multiprocessing import Process, Pipe
import time

class DockerWorker:
    def __init__(self, container_name, init_command):
        self.container_name = container_name
        self.client = docker.from_env()
        self.container = self.client.containers.get(self.container_name)
        self.bash = pexpect.spawn('docker exec -it {} bash'.format(self.container.id))
        self.bash.sendline(init_command)  # Start init_command when DockerWorker is initialized
        time.sleep(1)

    def execute_commands(self, commands):
        for cmd in commands:
            print(self.execute_command1(cmd))

    def execute_command2(self, cmd):
        self.bash.sendline(cmd)
        output = ''
        while True:
            try:
                self.bash.expect('\r\n', timeout=1)
                output += self.bash.before.decode('utf-8') + '\r\n'
            except pexpect.EOF:
                break
        return output

    def execute_command1(self, cmd):
        self.bash.sendline(cmd)
        print('cmd: >>>', cmd, '<<<')
        self.bash.expect('0\r\n', timeout=1)
        #self.bash.expect('\r\n', timeout=1)
        #self.bash.expect([pexpect.EOF], timeout=1)
        time.sleep(1)
        output = self.bash.before.decode('utf-8')
        print('output: >>>', output, '<<<')
        return output

    def execute_command3(self, cmd):
        self.bash.sendline(cmd)
        print('cmd: >>>', cmd, '<<<')

        output = []
        while True:
          line = self.bash.expect([pexpect.EOF], timeout=1)
          if line == 0:
            break
          output.append(line)

        #self.bash.expect('\r\n', timeout=10)
        output = self.bash.before.decode('utf-8')
        print('output: >>>', output, '<<<')
        return output

    def execute_command4(self, cmd):
        self.bash.sendline(cmd)
        print('cmd: >>>', cmd, '<<<')
        try:
            self.bash.expect(pexpect.EOF, timeout=1)
        except pexpect.EOF:
            pass
        except pexpect.TIMEOUT:
            pass
        #self.bash.expect('\r\n', timeout=1)
        #self.bash.expect([pexpect.EOF], timeout=1)
        time.sleep(1)
        output = self.bash.before.decode('utf-8')
        print('output: >>>', output, '<<<')
        return output

if __name__ == '__main__':
    # Specify the commands to run in each container
    commands_list = [
#        ['<{inst1} --> plant>.','done --> done.'],
        ['<{inst1} --> plant>.', '<{inst1} --> [green]>.', '<plant --> [green]>?'],
#        ['<{inst2} --> plant>.', '<{inst2} --> [green]>.', '<plant --> [green]>?'],
    ]

    init_command = '/app/NAR shell'
    workers = [DockerWorker('ONA', init_command) for _ in commands_list]


    # Start a worker process for each set of commands
    for i in range(len(workers)):
        Process(target=workers[i].execute_commands, args=(commands_list[i],)).start()

