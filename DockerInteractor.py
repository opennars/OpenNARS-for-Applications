import subprocess
import threading
import queue

class DockerInteractor:
    def __init__(self, command):
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,
            universal_newlines=True
        )

        self.stdout_queue = queue.Queue()
        self.stderr_queue = queue.Queue()

        # Start reader threads
        threading.Thread(target=self.reader, args=(self.process.stdout, self.stdout_queue)).start()
        threading.Thread(target=self.reader, args=(self.process.stderr, self.stderr_queue)).start()

    @staticmethod
    def reader(stream, queue):
        for line in iter(stream.readline, ''):
            queue.put(line)
        stream.close()

    def send(self, message):
        self.process.stdin.write(message + '\n')
        self.process.stdin.flush()

    def recv(self, timeout=None):
        try:
            return self.stdout_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def recv_err(self, timeout=None):
        try:
            return self.stderr_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def execute_command(self, command, delimiter):
        """Send a command and receive the output until a delimiter is seen."""
        self.send(command)
        
        output = ""
        while True:
            line = self.recv(1)
            if line is not None:
                output += line

            if line is None or delimiter in line:
                break

        return output.replace(delimiter, "")

    def shutdown(self):
        # Close the stdin stream
        if self.process.stdin:
            self.process.stdin.close()

        # Terminate the process
        self.process.terminate()

        # Wait for the process to terminate, to free system resources
        self.process.wait()

if __name__ == "__main__":
    docker_interactor = DockerInteractor(['docker', 'exec', '-i', 'ONA', '/app/NAR', 'shell'])
    iterations = 2000 
    for i in range(0, iterations):
        response = docker_interactor.execute_command("<cat --> furry_animal>.\n<cat --> furry_animal>?\n0\n", "done with 0 additional inference steps")
        print(f"Response: {response}")
    print(f"Done with {iterations} iterations")
    docker_interactor.shutdown()
