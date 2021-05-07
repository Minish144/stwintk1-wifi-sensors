const express = require('express')
const app = express()
const port = 65432

app.get('/', (req, res) => {
    console.log('here!')
  res.send('Hello World!')
})

app.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`)
})
