var ws = null;

// For colors: medialab.github.io/iwanthue/
var colors = ["#ff8673", "#001f5d", "#ffbd70", "#bd87ff", "#e1ffc9", "#9f001f", "#00977e", "#ffc4d2", "#003933", "#e5f1ff"]
var variables = new Array();
var startTime = Date.now()/1e3;
var time = new Array();
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
        // When the zeroth variable is recieved, use that to keep track of time
        if(varNum == 0) {
            time.push(Date.now()/1e3 - startTime);
        }

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


        // Setup graph
        if(u.series.length -1 == num) { // TODO: handle potentially skipped numbers
            u.addSeries({stroke: colors[num % colors.length], label:variables[num].name}, num+1);
        }
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
    ws = new WebSocket('ws://'+document.location.host+'/ws',['arduino']);

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






// TODO: uplot: add lines, add timing?, colors, titles




function getData(min) {
    let data = [];

    let l = time.length;
    time.splice(0, l - length);
    data.push(time);

    for( let i=0; i < variables.length; i++) {
        let l = variables[i].hist.length;
        variables[i].hist.splice(0, l - length);

        data.push(variables[i].hist);
    }
    return data;
}

const opts = {
    title: "Data",
    width: 1000,
    height: 600,
    pxAlign: false,
    scales: {
        x: {
            time:false,
            dir:1,
        },
        y: {
               auto: true,
        }
    },
    axes: [
        {
            space: 100,
        }
    ],
    series: [
        {},
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



