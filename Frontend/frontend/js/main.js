$( document ).ready(function() {
	getNodes();
	getSignals(0);
});

var nodes = new Array();
var signals = new Array();
var map = null;

function getNodes() {
	$.getJSON( "http://127.0.0.1:8000/csse4011_api/Nodes/", addNodes);
}

function addNodes(data) {

	$.each( data, function( key, val ) {
		var node = val.fields;
		node.active = "no";
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
	showNodes();
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
	var latitude = nodes[0].latitude;
	var longitude = nodes[0].longitude;
	var mapZoom = 20;
	var width = $( "#mapDiv" ).width();
	var height = $( "#mapDiv" ).height();
	var mapOptions = {
          center: new google.maps.LatLng(parseFloat(latitude), parseFloat(longitude)),
          zoom: mapZoom
        };
        map = new google.maps.Map(document.getElementById('mapDiv'),
            mapOptions);
	
}

function showSignals() {

}

function showCarEstimate() {

}


function updateNodesList() {
	$( "#nodesDiv" ).html("<h2>Nodes:</h2><br><br>");
	$.each(nodes, function( key, node ) {
		$( "#nodesDiv" ).append("Node " + node.Node_ID + ":<br>" +
								"		Latitude: " + node.latitude + "<br>" +
								"		Longitude: " + node.longitude + "<br>" +
								"		Active: " + node.active + "<br><br>");
	});
}



function showNodes() {

	 var nodeImage = {
		url: 'res/images/Node.png',
		size: new google.maps.Size(32, 32)
	  };
	  
	  //var nodeImage = 'res/images/Node.png';

	$.each(nodes, function( key, node ) {
		var nodeLatLng = new google.maps.LatLng(node.latitude, node.longitude);
		var beachMarker = new google.maps.Marker({
			position: nodeLatLng,
			map: map,
			icon: nodeImage
		});
	});
	updateNodesList();
}




var lastFrame = 0;
var carEstimates = []
var carsDrawn = []
var showEstimates = 1;
var showFilteredEstimates = 1;


function pollForCars() {
	$.getJSON( "http://127.0.0.1:8000/csse4011_api/VehicleEstimates/" + lastFrame + "/", addCars);
}

function addCars(data) {
	$.each( data, function( key, val ) {
		var estimate = val.fields;
		carEstimates.push(estimate);
		console.log("car added - Frame: " + estimate.FrameNum);
		lastFrame=estimate.FrameNum;

		drawCars();
	});
}


function drawCars() {
	$.each(carEstimates, function( key, estimate) {
		if (showEstimates && carsDrawn.indexOf(estimate.FrameNum) < 0) {
			drawEstimate(0, estimate);
		}
		if (showFilteredEstimates && carsDrawn.indexOf(estimate.FrameNum) < 0){
			drawEstimate(1, estimate);
		}
	});
}

function redrawCars() {
	
}

function drawEstimate(estimateType, estimate) {

	if (!estimateType) {	// estimates
		var estimateLatLng = new google.maps.LatLng(estimate.latitude, estimate.longitude);
		var marker = new google.maps.Marker({
		    position: estimateLatLng,
		    icon: {
		      path: google.maps.SymbolPath.CIRCLE,
		      scale: 4,
		      strokeColor: 'green'
		    },
		    draggable: false,
		    map: map
		  });

	} else {				// filtered Estimates
		var estimateFilteredLatLng = new google.maps.LatLng(estimate.latitudeFiltered, estimate.longitudeFiltered);
		var marker = new google.maps.Marker({
		    position: estimateFilteredLatLng,
		    icon: {
		      path: google.maps.SymbolPath.CIRCLE,
		      scale: 4,
		      strokeColor: 'red'
		    },
		    draggable: false,
		    map: map
		  });
	}

}


function startTCPAction() {
	$.getJSON( "http://127.0.0.1:8000/csse4011_api/StartTcpClient/", function() {
		//nothing
	});
	// start polling
	setInterval(pollForCars, 2000);
}




