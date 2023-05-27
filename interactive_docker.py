import DockerWorker  # Assuming DockerWorker is in the same directory

class InteractiveDocker:
    def __init__(self, container_name, init_command):
        self.worker = DockerWorker.DockerWorker(container_name, init_command)

    def run(self):
        while True:
            user_input = input("\nEnter command: ")

            # exit loop if user_input is 'exit'
            if user_input.lower() == 'exit':
                print("Exiting interactive mode...")
                break

            # send input to docker container and get response
            response = self.worker.execute_command4(user_input)

            # print response
            #print(response)


if __name__ == "__main__":
    container_name = "ONA"
    init_command = "/app/NAR shell"
    interactive_docker = InteractiveDocker(container_name, init_command)
    interactive_docker.run()

