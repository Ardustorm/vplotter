

var output;
var time = [];
var maxTimeLength = 80;

var variables = {};

// Reference for colors:
//   https://github.com/matplotlib/matplotlib/tree/master/lib/matplotlib/mpl-data/stylelib
var colors = ['#E24A33', '#348ABD', '#988ED5', '#777777', '#FBC15E', '#8EBA42', '#FFB5B8'];

var defaultData = {
    name : "",
    hist : [],
    maxLength : 50,
    color : "black",
    scale : 1,
    offset : 0
};


function configDataSetup() {
    var table = document.getElementById("dataConfig");
    var row = table.insertRow(0);

    var cell = row.insertCell(-1);
    cell.innerHTML = "Name";

    for(var item in defaultData) {
	if(item == "hist") {continue;}
	cell = row.insertCell(-1);
	cell.innerHTML = item;
    }
    cell = row.insertCell(-1);
    cell.innerHTML = "Set Var";
}


function processVariablePacket(data) {
    
    console.log("VARIABLE PACKET: ", data);
    
}


function createNewVariable(name) {
    var newData = {};
    for (var i in defaultData) {
	if( i == "color") {
	    newData[i] = colors[Object.keys(variables).length % colors.length];
	} else if(i != "hist") {
	    newData[i] = defaultData[i];
	} else {
	    newData[i] = [];  /* work around to pervent shallow copy */
	}
    }
    newData.name = name;
    variables[name] = newData;

    /* create interface */
    var table = document.getElementById("dataConfig");
    var row = table.insertRow(-1);
    var cell = row.insertCell(-1);
    cell.innerHTML = name;
    
    for(var item in defaultData) {
	if(item == "hist") {continue;}
	var cell = row.insertCell(-1);
	if(item == "name") {
	    cell.innerHTML = defaultData[item];
	} else {
	    cell.innerHTML = '<input type="text" size="10" value="'+ defaultData[item] +'" oninput="updateVariable(\''+ name +'\', \''+ item +'\', this.value)">';
	}
    }
    var cell = row.insertCell(-1);
    cell.innerHTML = '<input type="text" size="10" value="'+ 0 +'" onfocusout="setVariable(\''+ name +'\', this.value)">';

    
}

function updateVariable(name, item, value) {
    variables[name][item] = value;
    if(item == "maxLength") {
	variables[name].hist = variables[name].hist.slice(-value);
    }
}

function setVariable( name, value) {
    websocket.send("v " + name + " " + value +",");
}


function plotVariable(c, data, variableIndex) {
    var ctx = c.getContext("2d");
    if(data.hist.length < 2 || /* Not enough data to plot, so skip */
       data.maxLength <= 1) {  /* No data to plot */
	return;
    }
    var step = c.width / (data.maxLength-1);

    ctx.fillStyle = data.color;
    ctx.fillText(data.name + " " + data.hist[data.hist.length-1], 20, 20 + 20*variableIndex);


    /* we start at the last point and move backwards */
    ctx.beginPath();
    ctx.strokeStyle = data.color;
    ctx.moveTo(c.width, c.height - (parseFloat(data.offset) +
				    parseFloat(data.hist[data.hist.length-1])*data.scale));
    var dataGap = data.maxLength - data.hist.length;
    for( var i = data.hist.length-1; i >= 0; i--) {
	ctx.lineTo( (i + dataGap) * step,
		    c.height - (parseFloat(data.offset) + parseFloat(data.hist[i])*data.scale));
    }
    ctx.stroke();

}


function graph() {
    var c = document.getElementById("variableGraph");
    var ctx = c.getContext("2d");

    ctx.clearRect(0, 0, c.width, c.height);
    var variableIndex = 0;
    for( variable in variables) {
	var data = variables[variable];
	plotVariable(c, data, variableIndex++);

    }

    
}

