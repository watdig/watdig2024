const express = require('express');
const { exec } = require('child_process');

const app = express();
const port = 3000;

app.all('/start-script', (req, res) => {
  if (req.method === 'POST' || req.method === 'GET') {
      exec('cd C:\\Users\\yashp\\watdig2024\\ && node sshstart.js', (error, stdout, stderr) => {
          if (error) {
              console.error(`exec error: ${error}`);
              res.status(500).send('Error executing script');
              return;
          }
          console.log(`stdout: ${stdout}`);
          console.error(`stderr: ${stderr}`);
          res.send('Script started');
      });
  } else {
      res.status(405).send('Method Not Allowed');
  }
});

app.all('/end-script', (req, res) => {
  if (req.method === 'POST' || req.method === 'GET') {
      exec('cd C:\\Users\\yashp\\watdig2024\\ && node sshkill.js', (error, stdout, stderr) => {
          if (error) {
              console.error(`exec error: ${error}`);
              res.status(500).send('Error executing script');
              return;
          }
          console.log(`stdout: ${stdout}`);
          console.error(`stderr: ${stderr}`);
          res.send('Script started');
      });
  } else {
      res.status(405).send('Method Not Allowed');
  }
});

app.listen(port, () => {
  console.log(`Server is running on port ${port}`);
});