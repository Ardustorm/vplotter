

function sendPacketButton() {
    console.log(document.getElementById("test1").value);
    createNewVariable(document.getElementById("test1").value);
}


function createNewVariable(name) {
    var newData = {};
    newData.name = name;


    /* create interface */
    var table = document.getElementById("updateVariable");
    var row = table.insertRow(-1);
    var cell = row.insertCell(-1);
    cell.innerHTML = name;

    var cell = row.insertCell(-1);
    cell.innerHTML = '<input type="text" size="10" value="'+ 0 +'" onfocusout="setVariable(\''+ name +'\', this.value)">';
}

function btn1Run() {
    var table = document.getElementById("test1");
}


function addNewVariable(name) {
    var table = document.getElementById("updateVariable");
    var row = table.insertRow(-1);

    var cell = row.insertCell(-1);
    cell.innerHTML = name;

    var cell = row.insertCell(-1);
    cell.innerHTML = 34;

    var cell = row.insertCell(-1);
    cell.innerHTML ='<input class="myText" type="number" id="updateVar_' + name + '" placeholder="0" size="4">'+
        '<button id= "updateVarBtn_' + name + '" + onclick="console.log(\''+name+ '\')" class="btn">update</button>'+
        '</td></tr>';
}








function getData(min) {
    let xs = [];
    let ys = [];

    for (let i = min; i < min + length; i++) {
        xs.push(now + i/1000);
        ys.push(Math.sin(i/16) * 5);
    }

    return [
        xs,
        ys,
        _1,
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
            //    auto: false,
            range: [-6, 6],
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



