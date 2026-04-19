let map;
let Drones = [];
let Targets = [];
let lineCoordinates = [];
let lines = [];


function initMap() {

    // bu kısımda ground ve droneların çeşitli durumlardaki iconlarını oluşturuyoruz.    
    let Drone_icon = {
        url: "https://image.flaticon.com/icons/svg/215/215736.svg",
        scaledSize: new google.maps.Size(35, 35),
        anchor: new google.maps.Point(18, 18)
    };

    let Free = {
        url: "https://image.flaticon.com/icons/png/512/276/276300.png",
        scaledSize: new google.maps.Size(40, 40),
        anchor: new google.maps.Point(25, 25)
    };

    let Visited = {
        url: "https://image.flaticon.com/icons/png/512/276/276669.png",
        scaledSize: new google.maps.Size(40, 40),
        anchor: new google.maps.Point(25, 25)
    };

    let Visiting = {
        url: "https://image.flaticon.com/icons/png/512/277/277297.png",
        scaledSize: new google.maps.Size(40, 40),
        anchor: new google.maps.Point(25, 25)
    };

    let Ground_icon = {
        url: "https://image.flaticon.com/icons/svg/22/22819.svg",
        scaledSize: new google.maps.Size(40, 40),
        anchor: new google.maps.Point(20, 20)
    };

    let icons = { "Free": Free, "Visiting": Visiting, "Visited": Visited };

    
    $.getJSON("/static/json/target.json", function (target) {
        var i = 0;
        while (i < target.length) {
            Targets[i] = new google.maps.Marker({
                position: target[i],
                icon: icons[target[i].state],
                map: map // hangi haritaya konulacağı bilgisi
            });
            i++;
        }
    });
    
    
    // Uygulaması tarafından güncellenen drone.json üzerinden ground konum bilgisini alıyoruz.
    // Ground ip'si 10.0.0.1 oldugu için haritayı bu ip li konumda çağırıyoruz.
    $.getJSON("/static/json/drone.json", function (drone) {
        var i = 0;
        while (i < drone.length) {
            if (drone[i].ip == "10.0.0.1") {
                //map_state = map[m].lat + map[m].lng;
                map = new google.maps.Map(document.getElementById('map'), { // "map" id'li element haritayı içine yerleştirmek istediğimiz elementtir
                    center: { "lat": drone[i].lat, "lng": drone[i].lng }, // burada enlem ve boylam bilgilerini giriyoruz
                    zoom: 17 - drone.length + 2,
                    mapTypeId: 'roadmap'
                });
                // iconu haritaya koyan kısım
                // Ground konumunu 10.0.0.1 ipsine sahip olan enlem ve boylam bilgisine göre alıp
                // haritada iconunu set ediyoruz
                Drones[i] = new google.maps.Marker({
                    position: drone[i],
                    icon: Ground_icon,
                    title: "Ground",
                });
                // icon etrafında daire oluşturmak için çok gerekli değil
                var Circle = new google.maps.Circle({
                    strokeColor: "red",
                    strokeOpacity: 0.8,
                    strokeWeight: 2,
                    fillColor: "red",
                    fillOpacity: 0.25,
                    map: map,
                    center: drone[i],
                    radius: (200 * 0.9) * (drone.length - 1)
                });
            }
            else {
                Drones[i] = new google.maps.Marker({
                    position: drone[i],
                    icon: Drone_icon,
                    title: "Drone",
                });
            }
            i++;
        }
        let k = 0
        for (let i = 0; i < drone.length - 1; i++) {
            for (let j = i + 1; j < drone.length; j++) {
                if (getDistance(drone[i], drone[j]) < 500) {
                    var color = "#00FF00"
                    if (getDistance(drone[i], drone[j]) > (500 * 0.8)) {
                        color = "#FF0000";
                    }
                    lineCoordinates[0] = { lat: drone[i].lat, lng: drone[i].lng };
                    lineCoordinates[1] = { lat: drone[j].lat, lng: drone[j].lng };
                    lines[k] = new google.maps.Polyline({
                        path: lineCoordinates,
                        geodesic: true,
                        strokeColor: color,
                        strokeOpacity: 1.0,
                        strokeWeight: 2,
                    });
                    lines[k].setMap(map);
                    k++;
                }
            }
        }
    });    
}

var myVar = setInterval(bibib, 100);
function bibib() {
    clearDrones();
    clearTargets() 
    changeDronePosition();
    changeLinesPosition();
    changeTargetPosition()
    showDrones();
    showTargets();
}

function changeDronePosition() {
    $.getJSON("/static/json/drone.json", function (drone) {
        var i = 0;
        while (i < drone.length) {
            latlng = new google.maps.LatLng(drone[i].lat, drone[i].lng);
            Drones[i].setPosition(latlng);
            i++;
        }
    });
}

function changeTargetPosition() {
    $.getJSON("/static/json/target.json", function (target) {
        var i = 0;
        while (i < target.length) {
            latlng = new google.maps.LatLng(target[i].lat, target[i].lng);
            Targets[i].setPosition(latlng);
            i++;
        }
    });
}


function changeLinesPosition() {   
    $.getJSON("/static/json/drone.json", function (drone) {
    for(var i = 0 ; i < lines.length; i++ )
    {
        lines[i].setMap(null);
    }
    lines = []
        let k = 0
        for (let i = 0; i < drone.length - 1; i++) {
            for (let j = i + 1; j < drone.length; j++) {
                if (getDistance(drone[i], drone[j]) < 500) {
                    var color = "#00FF00"
                    if (getDistance(drone[i], drone[j]) > (500 * 0.8)) {
                        color = "#FF0000";
                    }
                    lineCoordinates[0] = { lat: drone[i].lat, lng: drone[i].lng };
                    lineCoordinates[1] = { lat: drone[j].lat, lng: drone[j].lng };
                    lines[k] = new google.maps.Polyline({
                        path: lineCoordinates,
                        geodesic: true,
                        strokeColor: color,
                        strokeOpacity: 1.0,
                        strokeWeight: 2,
                    });
                    lines[k].setMap(map);
                    k++;
                }
            }
        }
    });
}

function setMapOnAllDrones(map) {
    for (let i = 0; i < Drones.length; i++) {
        Drones[i].setMap(map);
    }
}
function setMapOnAllTargets(map) {
    for (let i = 0; i < Targets.length; i++) {
        Targets[i].setMap(map);
    }
}
// Removes the markers from the map, but keeps them in the array.
function clearDrones() {
    setMapOnAllDrones(null);
}
function clearTargets() {
    setMapOnAllDrones(null);
}

// Shows any markers currently in the array.
function showDrones() {
    setMapOnAllDrones(map);
}

function showTargets() {
    setMapOnAllTargets(map);
}

var rad = function (x) {
    return x * Math.PI / 180;
};

function getDistance(p1, p2) {
    var R = 6378137;
    var dLat = rad(p2.lat - p1.lat);
    var dLong = rad(p2.lng - p1.lng);
    var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
        Math.cos(rad(p1.lat)) * Math.cos(rad(p2.lat)) *
        Math.sin(dLong / 2) * Math.sin(dLong / 2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    var distance = R * c;
    return distance;
};


