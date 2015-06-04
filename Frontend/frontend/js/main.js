$( document ).ready(function() {
	getNodes();
	getSignals(0);
});

var nodes = new Array();
var signals = new Array();

var proj = new MercatorProjection();

//getCorners(google.maps.LatLng(nodes[0].latitude, nodes[0].longitude),mapZoom,640,640);

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
	
	console.log(nodes.length + " nodes added");
	loadMap();
}


function getSignals(timestamp) {
	$.getJSON( "http://127.0.0.1:8000/csse4011_api/Signals/" + timestamp + "/", addSignals);
}

function addSignals(data) {
	$.each( data, function( key, val ) {
		var signal = val.fields;
		signals.push(signal);
		var debugString = "Signal added - Node_ID: " +
			signal.Node_ID +
			"   Angle: " +
			signal.Angle +
			"   Intensity: " +
			signal.Intensity +
			"   Timestamp: " +
			signal.Timestamp;
		console.log(debugString);
	});
	console.log(signals.length + " signals added");
	//showSignals();
}






function loadMap() {
	$("#mapDiv").html(function() {
	var latitude = nodes[0].latitude;
	var longitude = nodes[0].longitude;
	var mapScale = 2;
	var mapZoom = 20;
	var width = $( "#mapDiv" ).width();
	var height = $( "#mapDiv" ).height();
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


function getCorners(center,zoom,mapWidth,mapHeight){
    var scale = Math.pow(2,zoom);
    var centerPx = proj.fromLatLngToPoint(center);
    var SWPoint = {x: (centerPx.x -(mapWidth/2)/ scale) , y: (centerPx.y + (mapHeight/2)/ scale)};
    var SWLatLon = proj.fromPointToLatLng(SWPoint);
    alert('SW: ' + SWLatLon);
    var NEPoint = {x: (centerPx.x +(mapWidth/2)/ scale) , y: (centerPx.y - (mapHeight/2)/ scale)};
    var NELatLon = proj.fromPointToLatLng(NEPoint);
    alert(' NE: '+ NELatLon);
}

