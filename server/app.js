const express = require('express')
const app = express()
let bodyParser = require('body-parser')

// Permit the app to parse application/x-www-form-urlencoded
app.use(bodyParser.urlencoded({ extended: false }));

// Use body-parser as middleware for the app.
app.use(bodyParser.json());

const port = 65432

app.get('/', function(req, res) {
    res.send('Hello World!');
    console.log('Req Ip: ', req.ip, 'Req body:', req.body);
});

app.get('/', function(req, res) {
    res.send('Hello World!');
    console.log('Req Ip: ', req.ip, 'Req body:', req.body, req.headers);
});

app.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`);
})