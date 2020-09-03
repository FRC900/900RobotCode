import paramiko

class JetsonSSH(object):

    def __init__(self, host='10.9.0.8'):
        self.host = host
        self.ssh = paramiko.SSHClient()
        self.user = 'ubuntu'
        self.pwd = 'ubuntu'


    def connect(self):
        self.ssh.connect(
            self.host, 
            username=self.user, 
            password=self.pwd,
            get_pty=True
        )

    def disconnect(self):
        self.ssh.close()

    def reboot(self):
        stdin, _, __ = self.ssh.exec_command('sudo reboot')
        stdin.write(self.pwd + '\n')
        stdin.flush()

if __name__ == "__main__":
    client = JetsonSSH()
    client.connect()
    _, stdout, __ = client.ssh.exec_command('ls')
    print(stdout.read())
    client.disconnect()
    
    
