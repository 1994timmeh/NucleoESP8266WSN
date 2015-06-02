$( document ).ready(getNodes);

var nodes = new Array();


function getNodes() {
	$.getJSON( "http://127.0.0.1:8000/csse4011_api/Nodes/", addNodes);
}


function addNodes(data) {
	if (data.length >= 1) {
		$("#test").append("<ul>");
	}
	$.each( data, function( key, val ) {
		var node = val.fields;
		nodes.push(node);
		var debugString = "Node added - ID: " +
			node.Node_ID +
			"   Latitude: " +
			node.latitude +
			"   Longitude: " +
			node.longitude;
		console.log(debugString);	
		$("#test").append("<li>" + debugString + "</li>");
	});
	
	if (data.length >= 1) {
		$("#test").append("</ul>");
	}
}
