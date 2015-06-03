$( document ).ready(getNodes);

var nodes = new Array();



function getNodes() {
	$.getJSON( "http://127.0.0.1:8000/csse4011_api/Nodes/", addNodes);
}


function addNodes(data) {

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
	});
	

	loadMap();
}


function loadMap() {
	$("#mapDiv").html(function() {
	var latitude = nodes[0].latitude;
	var longitude = nodes[0].longitude;
	var width = $( "#mapDiv" ).width();
	var height = $( "#mapDiv" ).height();
	var mapScale = 2;
	var mapZoom = 20;
	return "<img id=\"mapImg\" src=\"https://maps.googleapis.com/maps/api/staticmap?center=" + 
							latitude +
							',' +
							longitude +
							"&zoom="+ mapZoom +"&scale="+ mapScale +"&size=" + width + "x" + height + "\"></img>";
	});
	/* $.each(nodes, function() {
		latlng = getPixel(latitude, longitude);
		$("body").append(
            $('<div></div>')
                .css('position', 'absolute')
                .css('top', getPixel(latitude)[0])
                .css('left', mouseX + 'px')
                .css('width', size)
                .css('height', size)
                .css('background-color', color)
        );
	}); */
}


function getPixel(latitude, longitude) {
	
}


