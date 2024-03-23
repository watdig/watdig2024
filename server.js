const { Client } = require('ssh2');
const fs = require('fs');

// Create a new SSH client
const sshClient = new Client();

// Define connection parameters
const connectionParams = {
  host: '172.20.10.13',
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

  // Execute the command
  const command = process.platform === 'win32' ? 'cd /home/yashros-vm/watdig2024 && ./cleanbuild.sh' : './cleanbuild.sh';

  sshClient.exec(command, (err, stream) => {
    if (err) {
      console.error('Error executing command:', err.message);
      sshClient.end();
      return;
    }

    // Handle command output
    stream.on('data', data => {
      console.log('Output:', data.toString());
    }).on('close', () => {
      console.log('Command execution completed');
      sshClient.end();
    });
  });
});

sshClient.on('error', err => {
  console.error('SSH connection error:', err.message);
  sshClient.end();
});

sshClient.on('end', () => {
  console.log('SSH connection closed');
});

//"C:\\Program Files\\PuTTY\\plink.exe" -i "C:\\Users\\yashp\\.ssh\\id_rsa_putty1.ppk" -P 22 yashros-vm@172.20.10.13