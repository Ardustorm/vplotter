

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

















