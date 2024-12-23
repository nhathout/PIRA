const dgram = require('dgram');
const axios = require('axios');
const { Db } = require('tingodb')(); // TingoDB for local storage
const path = require('path');

// Create a UDP client
const client = dgram.createSocket('udp4');

// Define server details
const PORT = 41234;
const HOST = '192.168.0.167';

// Define the API endpoint for forwarding data
const API_ENDPOINT = 'http://127.0.0.1:8000/position';

// Set up TingoDB database
const dbPath = path.join(__dirname, 'data'); // Path to TingoDB directory
console.log(`TingoDB directory: ${dbPath}`);
const db = new Db(dbPath, {});
const positionsCollection = db.collection('positions');

// Message to send
const message = Buffer.from('ROBOTID 11');

// Function to send message to the server
function pingServer() {
    client.send(message, PORT, HOST, (err) => {
        if (err) {
            console.error(`Error sending message to server: ${err.message}`);
        } else {
            console.log(`Message sent to server: ${message}`);
        }
    });
}

// Listen for responses from the UDP server
client.on('message', async (msg, rinfo) => {
    console.log(`Received response from server: ${msg}`);

    // Parse the position data (assumes "ID,X,Z,theta,status" format)
    const positionData = parseMessage(msg.toString());
    console.log(`Parsed position data:`, positionData);

    // Save data to TingoDB
    positionsCollection.insert(positionData, (err, result) => {
        if (err) {
            console.error('Error saving to TingoDB:', err);
        } else {
            console.log('Data saved to TingoDB:', result);
        }
    });

    // Forward data to the FastAPI server
    try {
        const response = await axios.post(API_ENDPOINT, positionData);
        console.log(`Data successfully sent to API: ${response.status}`);
    } catch (error) {
        console.error(`Error sending data to API: ${error.message}`);
        if (error.response) {
            console.error('API Response:', error.response.data);
        }
    }
});

// Function to parse incoming messages
function parseMessage(message) {
    try {
        // Parse data in "ID,X,Z,theta,status" format
        const [robot_id, x, z, theta, status] = message.split(',');
        return {
            robot_id: parseInt(robot_id, 10),
            x: parseFloat(x),
            z: parseFloat(z),
            theta: parseFloat(theta),
            status: status.trim(),
        };
    } catch (error) {
        console.error('Error parsing message:', error);
        return { rawData: message }; // Return raw data if parsing fails
    }
}

// Query TingoDB to display stored data (optional)
function displayStoredData() {
    positionsCollection.find({}).toArray((err, docs) => {
        if (err) {
            console.error('Error querying TingoDB:', err);
        } else {
            console.log('All data stored in TingoDB:', docs);
        }
    });
}

// Start pinging the server every second
setInterval(() => {
    pingServer();
}, 1000);

// Optionally display stored data periodically for debugging
setInterval(() => {
    displayStoredData();
}, 5000);
