const express = require('express');
const { exec } = require('child_process');
const app = express();
const port = 3000;

app.post('/start-script', (req, res) => {
    exec('ssh yashros-vm@192.168.246.130./watdig2024/start.sh', (error, stdout, stderr) => {
        if (error) {
            console.error(`exec error: ${error}`);
            res.status(500).send('Error executing script');
            return;
        }
        console.log(`stdout: ${stdout}`);
        console.error(`stderr: ${stderr}`);
        res.send('Script started');
    });
});

app.listen(port, () => {
    console.log(`Server running at http://192.168.246.130:${port}`);
});