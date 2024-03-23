const { Client } = require('ssh2');
const fs = require('fs');

// Create a new SSH client
const sshClient = new Client();

// Define connection parameters
const connectionParams = {
  host: '192.168.2.87',
  port: 22,
  username: 'yashros-vm',
  privateKey: fs.readFileSync('C:\\Users\\yashp\\.ssh\\id_rsa'), // Corrected file path with escaped backslashes
  passphrase: '131404' // Provide the passphrase for your encrypted private key
};

// Connect to the SSH server
sshClient.connect(connectionParams);

// Handle SSH client events
sshClient.on('ready', () => {
  console.log('SSH connection established');

  // Open a shell for interaction after command execution
  sshClient.shell((err, stream) => {
    if (err) {
      console.error('Error opening shell:', err.message);
      sshClient.end();
      return;
    }

    const shellTimeout = setTimeout(() => {
      console.log('Shell session timeout reached. Ending session.');
      stream.end('exit\n'); // Send 'exit' command to end the session
    }, 2 * 60 * 1000);

    // Execute the command
    const command = 'cd /home/yashros-vm/watdig2024 && ./kill.sh';

    stream.write(command + '\n');

    // Handle command output
    stream.on('data', data => {
      console.log('Output:', data.toString());
    }).on('close', () => {
      console.log('Command execution completed');
      sshClient.end();
    });

    // Handle shell output
    process.stdin.pipe(stream);
    stream.pipe(process.stdout);
    stream.stderr.pipe(process.stderr);
  });


});

sshClient.on('error', err => {
  console.error('SSH connection error:', err.message);
  sshClient.end();
});

sshClient.on('end', () => {
  console.log('SSH connection closed');
});