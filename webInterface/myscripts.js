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
    


}

function findVal(ch, lst) {
    for( var i = 0, len = lst.length; i < len; i++) {
	if( lst[i][0] == ch ) {
	    return parseFloat(lst[i].substr(1));
	}
    }
    return null;
}

function evalGcode(s) {
    clean = s.toUpperCase().split(";")[0]; // make case insensitive, and remove comments
    var cmd = clean.trim().split(" ");

    switch( cmd[0] ) {
    case "G0" :
    case "G1" :
	// console.log("test: " + cmd);
	var x = findVal("X", cmd);
	var y = findVal("Y", cmd);
	if(x) curX = x;
	else console.log("failedX");
	if(y) curY = y;
	else console.log("failedY");
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

    drawingCtx.beginPath();
    drawingCtx.arc(curX, curY, 5, 0, 2 * Math.PI, true);
    drawingCtx.fillStyle = "#1F1F1F";
    drawingCtx.fill();
    drawingCtx.closePath();

    context.drawImage(drawingCan, 0, 0);

    
    updatePlotter(context, curX, curY);
    
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


    console.log("UPDATE:" + curX + ", " +curY + "     "+ x + ", " +y);
    ctx.fillStyle = "#11ff11";
    ctx.fillText(document.getElementById("width").value, x, y+35);
    ctx.fillStyle = "steelblue";

    // Line lengths
    la = Math.sqrt( Math.pow(x,2) + Math.pow(y,2));
    lb = Math.sqrt( Math.pow((canvas.clientWidth-x),2) + Math.pow(y,2));
    
    ctx.fillStyle = "white";
    ctx.fillText("███" , x/2, y/2);
    ctx.fillText("███" , (x+canvas.clientWidth)/2, y/2);
    ctx.fillStyle = "steelblue";


    ctx.fillText(+la.toFixed(2), x/2, y/2);
    ctx.fillText(+lb.toFixed(2) , (x+canvas.clientWidth)/2, y/2);
    console.log("pltr2: ("+ x +", "+ y +")");
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
