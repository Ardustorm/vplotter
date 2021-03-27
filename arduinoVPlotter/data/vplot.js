var ws = null;
var variables = new Array();
loop = 0;

function sendBlob(str){
    var buf = new Uint8Array(str.length);
    for (var i = 0; i < str.length; ++i) buf[i] = str.charCodeAt(i);
    ws.send(buf);
}



function processData(d) {
    var table = document.getElementById("dataConfig");
    var view = new DataView(d);
    var i = 1;
    while(i+1 < d.byteLength) {
        var varNum = view.getUint8(i);
        i++;

        // Request data if we don't currently have that varaiable number and exit
        if(varNum >= variables.length) {
            sendBlob("S");
            return; // Stop processing packet since we can't store all of the info
        }
        if (variables[varNum]["type"] == 'f') {
            variables[varNum]["value"] = view.getFloat32(i, true);
            variables[varNum].hist.push( variables[varNum].value);
        } else if (variables[varNum]["type"] == 'i') {
            variables[varNum]["value"] = view.getInt32(i, true);
            variables[varNum].hist.push( variables[varNum].value);
        } else {
            console.log("UNKNOWN DATA TYPE");
        }
        table.rows[varNum+1].cells[1].innerHTML = variables[varNum]["value"];
        i+=4;
        if(loop == 10) {
            start = console.time("callAPITimer");
        }
        if(loop == 10010) {
            console.timeEnd('callAPITimer')
        }
        loop++;
    }
}
function processSetup(d) {
    var table = document.getElementById("dataConfig");
    var arr = new Uint8Array(d)
    var i = 1;

    while(i < d.byteLength) {
        var num = arr[i++];
        var type = String.fromCharCode(arr[i++]);

        // TODO: Add ability to process type
        var str = String.fromCharCode.apply(String, new Uint8Array(d, i, (arr.indexOf(0,i))-i));
        i = arr.indexOf(0,i) + 1;

        // Populate table and array with data if needed
        while(variables.length <= num) {
            variables.push({"name": "N/A", "type":null, "value":NaN, "hist":[]});
            var row = table.insertRow(-1);
            row.innerHTML = "<td>N</td><td>v</td><td>"+
                "<input class='myText' type='number' step='any' id='updateVal"+num+"' size=6> " +
                "<button onclick='updateVariable("+num +
                ", document.getElementById(\"updateVal"+num+"\").value)'>update</button>" +
                "</td>";
        }
        // variables[num] = {"name": str, "type":type};
        variables[num].name = str;
        variables[num].type = type
        table.rows[num+1].cells[0].innerHTML = str;
    }
}
function updateVariable( varNum, value) {
    var a = new ArrayBuffer(6);
    var packet = new DataView(a);
    packet.setUint8(0, "V".charCodeAt(0), true);
    packet.setUint8(1, varNum, true);
    if(variables[varNum].type == 'f') {
        packet.setFloat32(2, value, true);
    } else if(variables[varNum].type == 'i') {
        packet.setInt32(2, value, true);
    }
    ws.send(a);
}
function testFunc(d) {
    var view = new DataView(d);
    var opcode = String.fromCharCode(view.getUint8(0));
    if(opcode == 'S'){
        processSetup(d);

    } else if(opcode == 'D') {
        processData(d);
        // TODO: Right now I'm getting about 330 Hz update rate, but I could probably improve
        // that by skipping the 'sendBlob' function and sending the binary data directly
        sendBlob("D");

    } else {
        console.log("unknown");
    }
}
function startSocket(){
    // ws = new WebSocket('ws://'+document.location.host+'/ws',['arduino']);
    ws = new WebSocket('ws://192.168.0.180/ws',['arduino']);
    ws.binaryType = "arraybuffer";
    ws.onerror = function(e){
        console.log("ws error", e);
    };
    ws.onmessage = function(e){
        var msg = "";
        if(e.data instanceof ArrayBuffer){
            // msg = "BIN:" +
            //      Array.prototype.map.call(new Uint8Array(e.data), x => ('00' + x.toString(16)).slice(-2)).join('');
            testFunc(e.data);
        } else {
            msg = "TXT:"+e.data;
        }
    };

}
function onBodyLoad(){
    startSocket();
}











function getData(min) {
    let xs = [];
    let ys = [];
    let ds = [];

    for (let i = min; i < min + length; i++) {
        xs.push(now + i/1000);
        ys.push(Math.sin(i/16) * 5);
    }
    for( let i=0; i < variables.length; i++) {
        let l = variables[i].hist.length;
        variables[i].hist.splice(0, l - length);

        ds = variables[0].hist;
    }



    return [
        xs,
        ys,
        ds,
        _2,
        _3,
        _4,
        _5,
    ];
}

const opts = {
    title: "6 series x 600 points @ 60fps",
    width: 1000,
    height: 600,
    pxAlign: false,
    scales: {
        x: {
            //    auto: false,
            //range: [-6, 6],
            time:true,
            dir:1,
        },
        y: {
               auto: true,
            // range: [-6, 6],
        }
    },
    axes: [
        {
            space: 300,
        }
    ],
    series: [
        {},
        {
            label: "Sine",
            stroke: "red",
            fill: "rgba(255,0,0,0.1)",
        },
        {
            stroke: "green",
            fill: "#4caf505e",
        },
        {
            stroke: "blue",
            fill: "#0000ff20",
        },
        {
            stroke: "orange",
            fill: "#ffa5004f",
        },
        {
            stroke: "magenta",
            fill: "#ff00ff20",
        },
        {
            stroke: "purple",
            fill: "#80008020",
        },
    ],
};

function update() {
    shift += 1;
    data = getData(shift);
    u.setData(data);
    requestAnimationFrame(update);
}


let length = 600;

let _1 = Array(length).fill(1);
let _2 = Array(length).fill(2);
let _3 = Array(length).fill(3);
let _4 = Array(length).fill(4);
let _5 = Array(length).fill(5);

let now = Math.floor(Date.now()/1e3);
let shift = 0;

let data = getData(shift);



