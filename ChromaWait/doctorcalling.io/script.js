/*
  QR Code generator and MQTT sender

  Draws a QR code using a text string. The QR code is the sketch's 
  URL. Also sends an MQTT message to shiftr.io.
  
  Uses 
  https://github.com/kazuhikoarase/qrcode-generator
  as the QR Code generator library. It's hosted at this CDN:
  https://unpkg.com/qrcode-generator@1.4.4/qrcode.js

  created 22 Aug 2020
  modified 23 Nov 2020
  by Tom Igoe
*/

// a string to d isplay in the QR code
// (the URL of this sketch):
let urlString = parent.location.href;
// an HTML div to display it in:


// MQTT broker details:

let broker = {
    hostname: 'voruin.cloud.shiftr.io',
    port: 443
};

// let broker = {
//     hostname: 'public.cloud.shiftr.io',
//     port: 443
// };


// MQTT client:
let client;
// client credentials:

let creds = {
    clientID: 'doctorClient',
    userName: 'voruin',
    password: 'lettherebelight'
}



// topic to subscribe to when you connect:
let topic = 'lights';

// a pushbutton to send messages
let sendButton;
// divs for the local and remote messages:
// let localDiv;
// let remoteDiv;

let allInfo;

function setup() {
    updateTime();
    // createCanvas(windowWidth, windowHeight);
    noCanvas();
    // Create an MQTT client:
    client = new Paho.MQTT.Client(broker.hostname, Number(broker.port), creds.clientID);
    // set callback handlers for the client:
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;
    // connect to the MQTT broker:
    client.connect(
        {
            onSuccess: onConnect,       // callback function for when you connect
            userName: creds.userName,   // username
            password: creds.password,   // password
            useSSL: true                // use SSL
        }
    );
    // create the send button:
    // sendButton = createButton('Submit');
    // sendButton.position(20, 150);


    // let button = document.getElementById("sendButton");
    // sendButton.addEventListener('mouseup', sendMqttMessage);
    // sendButton.addEventListener('mousedown', buttonPressed);

//     localDiv = createDiv('local messages will go here');
//     localDiv.position(30, 400);

//     remoteDiv = createDiv('waiting for messages');
//     remoteDiv.position(30, 430);

// let trytry =  document.getElementById("try");
// trytry.style.display = "none";

}

function draw() {

}

// // This function hides the tag div when you click on it:
// function hideTag() {
//     tagDiv.hide();
// }

// called when the client connects
function onConnect() {
//     localDiv.html('client is connected');
    client.subscribe(topic);
}

// called when the client loses its connection
function onConnectionLost(response) {
    if (response.errorCode !== 0) {
        // let connectionInfo= String('onConnectionLost:' + response.errorMessage);
        // localDiv.innerHTML = connectionInfo;
//         localDiv.html('onConnectionLost:' + response.errorMessage);
    }
}

// called when a message arrives
function onMessageArrived(message) {
    // let gotShow= String('I got a message:' + message.payloadString);
    // remoteDiv.innerHTML = gotShow;
//     remoteDiv.html('I got a message:' + message.payloadString);
    // let incomingNumber = parseInt(message.payloadString);
    // invert the message each time: 0, then 254, then 0, etc.:
    // if (incomingNumber > 0) {
    //     brightness = 0;
    // } else {
    //     brightness = 254;
    // }
    if (isJson(message.payloadString)){
    var patientData = JSON.parse(message.payloadString);
    addRow('patientTable', patientData.name, patientData.doctor, patientData.apptTime, patientData.subTime, patientData.color);
    // alert(patientData.name);
    // alert(patientData.doctor);
    // alert(patientData.apptTime);
    // alert(patientData.subTime);
    // alert(patientData.color);
    allInfo = push([patientData.name, patientData.doctor, patientData.apptTime, patientData.subTime, patientData.color]);
    }
}

// called when you want to send a message:
function sendMqttMessage(patientColor) {
    // if the client is connected to the MQTT broker:
    if (client.isConnected()) {
        // // make a string with a random number form 0 to 15:
        // let patientName = document.getElementById("fname").value + " " + document.getElementById("lname").value;
        // let doctor = document.getElementById("doctorName").value;
        // let reservationTime = document.getElementById("appt").value;

        // let randomColorHEX = Math.floor(Math.random() * 16777215).toString(16);
        // let colorDiv = document.getElementById("individualColor");
        // colorDiv.style.backgroundColor = "#" + randomColorHEX;

        // let randomColor = [hexToRgb(randomColorHEX).r, hexToRgb(randomColorHEX).g, hexToRgb(randomColorHEX).b]
        // // color.innerHTML = "#" + randomColorHEX;


        // let msg = JSON.stringify({ "name": patientName, "doctor": doctor, "apptTime": reservationTime, "subTime": timeNow, "color": randomColor });
        //   let msgR = String(patientColor[0])+","+String(patientColor[1])+","+String(patientColor[2]);
        let msgR =  String(patientColor[0]);
        let msgG =  String(patientColor[1]);
        let msgB =  String(patientColor[2]);

        // start an MQTT message:
        messageR = new Paho.MQTT.Message(msgR);
        // choose the destination topic:
        messageR.destinationName = "doctorCallingH";
        // send it:
        client.send(messageR);
        // print what you sent:
//         localDiv.html('I sent: ' + messageR.payloadString);
      
          // start an MQTT message:
          messageG = new Paho.MQTT.Message(msgG);
          // choose the destination topic:
          messageG.destinationName = "doctorCallingS";
          // send it:
          client.send(messageG);
          // print what you sent:
//           localDiv.html('I sent: ' + messageG.payloadString);

            // start an MQTT message:
        messageB = new Paho.MQTT.Message(msgB);
        // choose the destination topic:
        messageB.destinationName = "doctorCallingV";
        // send it:
        client.send(messageB);
        // print what you sent:
//         localDiv.html('I sent: ' + messageB.payloadString);
    }

}

function updateTime() {
    var date = new Date();
    var h = date.getHours();
    var m = date.getMinutes();
    var s = date.getSeconds();
    if (h < 10) h = "0" + h;
    if (m < 10) m = "0" + m;
    if (s < 10) s = "0" + s;
    timeNow = h + ":" + m + ":" + s;
    var timeToShow = h + ":" + m;
    // document.getElementById("appt").value = timeToShow;
}


// function hexToRgb(hex) {
//     var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
//     return result ? {
//         r: parseInt(result[1], 16),
//         g: parseInt(result[2], 16),
//         b: parseInt(result[3], 16)
//     } : null;
// }

function buttonPressed() {
    let buttonToChange = document.getElementById("sendButton");
    buttonToChange.style.background = "#e058a7";
    buttonToChange.style.color = "#ffffff";
}

function hexToRgb(hex) {
    var bigint = parseInt(hex, 16);
    var r = (bigint >> 16) & 255;
    var g = (bigint >> 8) & 255;
    var b = bigint & 255;

    return {
        r: r,
        g: g,
        b: b
    }
}

function addRow(tableID, cell1Text, cell2Text, cell3Text, cell4Text, patientColor) {
    // Get a reference to the table
    let tableRef = document.getElementById(tableID);

    // Insert a row at the end of the table
    let newRow = tableRef.insertRow(-1);
    // checkbox.classList.add('try');

    let newCell0 = newRow.insertCell(0);
    // let newText1 = document.createTextNode(cell1Text);
    var checkbox = document.createElement("INPUT");
     checkbox.type = "checkbox";
     newCell0.appendChild(checkbox);
     
     checkbox.addEventListener('change', function() {
        sendMqttMessage(patientColor);
        checkbox.setAttribute("disabled", "false");
        // enabledSettings = 
        //   Array.from(checkboxes) // Convert checkboxes to an array to use filter and map.
        //   .filter(i => i.checked) // Use Array.filter to remove unchecked checkboxes.
        //   .map(i => i.value) // Use Array.map to extract only the checkbox values from the array of objects.
          
        // console.log(enabledSettings)
      })

    // Insert a cell in the row at index 0
    let newCell1 = newRow.insertCell(1);

    // Append a text node to the cell
    let newText1 = document.createTextNode(cell1Text);
    newCell1.appendChild(newText1);

    let newCell2 = newRow.insertCell(2);
    let newText2 = document.createTextNode(cell2Text);
    newCell2.appendChild(newText2);

    let newCell3 = newRow.insertCell(3);
    let newText3 = document.createTextNode(cell3Text);
    newCell3.appendChild(newText3);

    let newCell4 = newRow.insertCell(4);
    let newText4 = document.createTextNode(cell4Text);
    newCell4.appendChild(newText4);

}

  // Call addRow() with the table's ID
//   addRow('my-table');
function isJson(str) {
    try {
        JSON.parse(str);
    } catch (e) {
        return false;
    }
    return true;
}
