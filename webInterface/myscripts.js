// TODO:
//  Scaling
//  Arcs
//  Ignore parenthesis (and maybe %)
//  Set origin / offset
var canvas = document.querySelector("#myCanvas");
var context = canvas.getContext("2d");
var curX = canvas.clientWidth/2;
var curY = canvas.clientHeight/2;
var delta = 1;
var canvasPos = getPosition(canvas);
var gcodeTxt ="";


var drawingCan = document.createElement('canvas');
drawingCan.width =  canvas.clientWidth;
drawingCan.height = canvas.clientHeight;
var drawingCtx = drawingCan.getContext('2d');


canvas.addEventListener("mousedown", buttonPress, false);

canvas.addEventListener("mousemove", setMousePosition, false);
canvas.addEventListener("mousemove", update, false);

function newGcode() {
    gcodeTxt = document.getElementById('gcode').value;
    var gcode = gcodeTxt.split("\n").filter(Boolean); // split newlines and ignore blanks
    for( var i = 0, len = gcode.length; i < len; i++) {
    	console.log(i +": "  + gcode[i]);
    	evalGcode(gcode[i]);
    }
    
    update();

}


function findVal(ch, lst) {
    for( var i = 0, len = lst.length; i < len; i++) {
	if( lst[i][0] == ch ) {
	    return parseFloat(lst[i].substr(1));
	}
    }
    console.log("Could not find: ", ch, " in: ", lst)
    return null;
}

function evalGcode(s) {
    clean = s.toUpperCase().split(";")[0]; // make case insensitive, and remove comments
    var cmd = clean.trim().split(" ");

    switch( cmd[0] ) {
    case "G0" :
    case "G1" :
    case "G00" :
    case "G01" :
    case "G02" :
    // case "G03" :
	// console.log("test: " + cmd);
	var x = findVal("X", cmd);
	var y = findVal("Y", cmd);
	if(x) curX = x;
	else console.log("failedX");
	if(y) curY = y;
	else console.log("failedY");
	break;
	
    // case "G02" :
    // 	var x0 = curX;
    // 	var y0 = curY;
    // 	var x = findVal("X", cmd);
    // 	var y = findVal("Y", cmd);
    // 	var i = findVal("I", cmd);
    // 	var j = findVal("J", cmd);
    // 	// Still needs work
    // 	var theta0 = Math.atan2(y0-j, x0-i);
    // 	var theta1 = Math.atan2(y-j, x-i);
    // 	var R = Math.sqrt( Math.pow(y0-j, 2) + Math.pow(x0-i,2));
    // 	for( var t = 0; t < 1; t+=.1) {
    // 	    curX = i + R * Math.cos(theta0 - t * (2*Math.PI -(theta1-theta0)) );
    // 	    curY = j + R * Math.sin(theta0 - t * (2*Math.PI -(theta1-theta0)) );
    // 	    update();
	    
    // 	}
    // 	break;
    case "G03" :
    	var x0 = curX;
    	var y0 = curY;
    	var x = findVal("X", cmd);
    	var y = findVal("Y", cmd);
    	var i = findVal("I", cmd);
    	var j = findVal("J", cmd);

    	var theta0 = Math.atan2(y0-j, x0-i);
    	var theta1 = Math.atan2(y-j, x-i);
    	var R = Math.sqrt( Math.pow(y0-j, 2) + Math.pow(x0-i,2));
    	for( var t = 0; t < 1; t+=.1) {
    	    curX = i + R * Math.cos(theta0 + t * (theta1-theta0));
    	    curY = j + R * Math.sin(theta0 + t * (theta1-theta0));
    	    update();
    	}
	
	break;
    default:
	console.log("unknown gcode: " + s);
    }

    update();
}

function buttonPress(e) {
    if (e.button == 0) {
	console.log("Left mouse button pressed!");
    } else if (e.button == 1) {
	console.log("Middle mouse button pressed!");
    } else if (e.button == 2) {
	console.log("Right mouse button pressed!");
	drawingCtx.clearRect(0, 0, canvas.width, canvas.height);
	update();
    }
}

function setMousePosition(e) {
    // based on AxFab: stackoverflow.com/questions/1114465
    // May need this if weird sizes on phone or something:
    // https://stackoverflow.com/q/17130395/

    var canvasPos = canvas.getBoundingClientRect();
    curX = e.clientX - canvasPos.left;
    curY = e.clientY - canvasPos.top;
}


update();

function update() {
    context.clearRect(0, 0, canvas.width, canvas.height);

    // updatePlotter(context, curX, curY-20);


    drawingCtx.globalAlpha = .7;

    // drawingCtx.beginPath();
    // drawingCtx.arc(curX, curY, 2, 0, 2 * Math.PI, true);
    // drawingCtx.fillStyle = "#1F1F1F";
    // drawingCtx.fill();
    // drawingCtx.closePath();

    context.drawImage(drawingCan, 0, 0);

    
    updatePlotter(context, curX, curY);


    // Line test
    if (curX != update.prevX ||
	curY != update.prevY) {
	drawingCtx.beginPath();
	drawingCtx.moveTo(update.prevX, update.prevY);
	drawingCtx.lineTo(curX*2,-curY*2+600);
	drawingCtx.stroke();
    }

    
    update.prevX = curX*2;
    update.prevY = -curY*2 +600;
    // requestAnimationFrame(update);
}


function updatePlotter(ctx, x, y) {
    // ctx.clearRect(0, 0, canvas.width, canvas.height);
    //circle
    ctx.beginPath();
    ctx.arc(x, y, 10, 0, 2 * Math.PI, true);
    ctx.fillStyle = "#FF6A6A";
    ctx.fill();
    ctx.closePath();
    //lines
    ctx.lineWidth = 3;
    ctx.strokeStyle = "#333";
    ctx.fillStyle = "steelblue";
    ctx.beginPath();
    ctx.moveTo(0,0);
    ctx.lineTo(x, y);
    ctx.lineTo(canvas.width, 0);
    ctx.stroke();

    //Text
    ctx.font = "20px Helvetica, Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.strokeStyle = "#f73b79";
    ctx.fillText("("+x+", "+y+")", x, y+15);


    ctx.fillText(document.getElementById("width").value, x, y+35);

    // Line lengths
    la = Math.sqrt( Math.pow(x,2) + Math.pow(y,2));
    lb = Math.sqrt( Math.pow((canvas.clientWidth-x),2) + Math.pow(y,2));
    
    ctx.fillStyle = "white";
    ctx.fillText("███" , x/2, y/2);
    ctx.fillText("███" , (x+canvas.clientWidth)/2, y/2);
    ctx.fillStyle = "steelblue";


    ctx.fillText(+la.toFixed(2), x/2, y/2);
    ctx.fillText(+lb.toFixed(2) , (x+canvas.clientWidth)/2, y/2);
}



// Helper function to get an element's exact position
function getPosition(el) {
    var xPos = 0;
    var yPos = 0;
    while (el) {
	xPos += (el.offsetLeft - el.scrollLeft + el.clientLeft);
	yPos += (el.offsetTop - el.scrollTop + el.clientTop);
	el = el.offsetParent;
    }

    return {
	x: xPos,
	y: yPos
    };
}
