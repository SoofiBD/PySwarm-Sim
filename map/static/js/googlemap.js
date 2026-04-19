var map;

function initMap() {

    var map_state;
    var m = 0;

    // Uygulaması tarafından güncellenen drone.json üzerinden ground konum bilgisini alıyoruz.
    // Ground ip'si 10.0.0.1 oldugu için haritayı bu ip li konumda çağırıyoruz.
    $.getJSON('/static/json/drone.json', function (map) {
        while (m < map.length) {
            if (map[m].ip == "10.0.0.1") {
                map_state = map[m].lat + map[m].lng;
                var map = new google.maps.Map(document.getElementById('map'), { // "map" id'li element haritayı içine yerleştirmek istediğimiz elementtir
                    center: { "lat": map[m].lat, "lng": map[m].lng }, // burada enlem ve boylam bilgilerini giriyoruz
                    zoom: 17 - map.length + 2,
                    mapTypeId: 'roadmap'
                });
                break;
            }
            m++;
        }

        // bu kısımda ground ve droneların çeşitli durumlardaki iconlarını oluşturuyoruz.
        var Drone_icon = {
            url: "https://image.flaticon.com/icons/svg/215/215736.svg",
            scaledSize: new google.maps.Size(35, 35),
            anchor: new google.maps.Point(18, 18)
        };

        var Free = {
            url: "https://image.flaticon.com/icons/png/512/276/276300.png",
            scaledSize: new google.maps.Size(40, 40),
            anchor: new google.maps.Point(25, 25)
        };

        var Visited = {
            url: "https://image.flaticon.com/icons/png/512/276/276669.png",
            scaledSize: new google.maps.Size(40, 40),
            anchor: new google.maps.Point(25, 25)
        };

        var Visiting = {
            url: "https://image.flaticon.com/icons/png/512/277/277297.png",
            scaledSize: new google.maps.Size(40, 40),
            anchor: new google.maps.Point(25, 25)
        };

        var icons = { "Free": Free, "Visiting": Visiting, "Visited": Visited };

        // popupclass dronelar arası mesafeyi göstermek için kullanılıyor.
        function createPopupClass() {
            function Popup(position, content) {
                this.position = position;
                content.classList.add('popup-bubble');
                var bubbleAnchor = document.createElement('div');
                bubbleAnchor.appendChild(content);
                this.containerDiv = document.createElement('div');
                this.containerDiv.classList.add('popup-container');
                this.containerDiv.appendChild(bubbleAnchor);
                google.maps.OverlayView.preventMapHitsAndGesturesFrom(this.containerDiv);
            }
            Popup.prototype = Object.create(google.maps.OverlayView.prototype);
            Popup.prototype.onAdd = function () {
                this.getPanes().floatPane.appendChild(this.containerDiv);
            };
            Popup.prototype.draw = function () {
                var divPosition = this.getProjection().fromLatLngToDivPixel(this.position);
                this.containerDiv.style.left = divPosition.x + 'px';
                this.containerDiv.style.top = divPosition.y + 'px';
            };
            return Popup;
        }

        // dronelar arası mesafeyi enlem ve boylam bilgisi kullanarak hesaplayan kod
        var rad = function (x) {
            return x * Math.PI / 180;
        };

        var getDistance = function (p1, p2) {
            var R = 6378137;
            var dLat = rad(p2.lat - p1.lat);
            var dLong = rad(p2.lng - p1.lng);
            var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(rad(p1.lat)) * Math.cos(rad(p2.lat)) *
                Math.sin(dLong / 2) * Math.sin(dLong / 2);
            var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
            var d = R * c;
            return d;
        };

        // genel değişkenler dronelar google map markerları olarak gözükecek tabi bizim seçtiğimiz iconlarla

        // 200 milisaniye ara ile haritayı güncelliyoruz
        var myVar = setInterval(myTimer, 200);
        var j = 0;
        var markers = [];
        var circles = [];
        var flightP = [];
        var targets = [];
        var PopArr = [];
        var Ddist;
        var D1;
        var D2;
        var popup;
        var Popup;
        var con_state = "red";
        var droneCount;
        //var tarLen;
        var targetStates = [];


        // target.json üzerinden hedeflerin enlem ve boylamları okunur
        // bu veriler javadan gelir
        $.getJSON('/static/json/target.json', function (tarfirstLoad) {
            document.getElementById("dis").innerHTML = tarfirstLoad.length;
            for (var t = 0; t < tarfirstLoad.length; t++) {
                var marker = new google.maps.Marker({
                    position: tarfirstLoad[t],//hedef konumu
                    icon: icons[tarfirstLoad[t].state], // hedef iconu
                    map: map
                });
                targets.push(marker);
                targetStates.push(tarfirstLoad[t]);
            }
        });

        var Ground_icon = {
            url: "https://image.flaticon.com/icons/svg/22/22819.svg",
            scaledSize: new google.maps.Size(40, 40),
            anchor: new google.maps.Point(20, 20)
        };

        // Ground konumunu 10.0.0.1 ipsine sahip olan enlem ve boylam bilgisine göre alıp
        // haritada iconunu set ediyoruz
        $.getJSON('/static/json/drone.json', function (ground) {
            droneCount = ground.length;
            for (var f = 0; f < droneCount; f++) {
                if (ground[f].ip == "10.0.0.1") {

                    // iconu haritaya koyan kısım
                    var Ground = new google.maps.Marker({
                        position: ground[f],
                        icon: Ground_icon,
                        map: map // hangi haritaya konulacağı bilgisi
                    });

                    // icon etrafında daire oluşturmak için çok gerekli değil
                    var Circle = new google.maps.Circle({
                        strokeColor: "red",
                        strokeOpacity: 0.8,
                        strokeWeight: 2,
                        fillColor: "red",
                        fillOpacity: 0.25,
                        map: map,
                        center: ground[f],
                        radius: 200 * (droneCount - 1)
                    });
                    break;
                }
            }
        });


        // Droneların aralarındaki mesafeleri yazmak için drone konumları popup nesnesi ile birlikte kullanılmış 
        $.getJSON('/static/json/drone.json', function (DRONE) {
            for (var b = 0; b < DRONE.length; b++) {

                var Newdist = document.createElement("div");
                Newdist.id = "DistM" + b;
                var node = document.createTextNode(DRONE[b].ip.substring(6, 8));
                Newdist.appendChild(node);
                var element = document.getElementById("display");
                element.appendChild(Newdist);
                document.getElementById("DistM" + b).style.color = "black";

                Popup = createPopupClass();
                popup = new Popup(
                    new google.maps.LatLng(DRONE[b].lat, DRONE[b].lng),
                    document.getElementById("DistM" + b));
                popup.setMap(map);
                //PopArr.push(popup);

            }
        });

        // belirli aralıklar ile çalışarak haritadaki konumları yenileyen method
        function myTimer() {


            // drone markerlar
            for (var k = 0; k < markers.length; k++) {
                markers[k].setMap(null);
            }
            /*
                        for (var k = 0; k < circles.length; k++) {
                            circles[k].setMap(null);
                        }
            */
            // Dronelar arası çizgiler
            for (var k = 0; k < flightP.length; k++) {
                flightP[k].setMap(null);
            }

            // dronelar arası mesafelerin yazıldığı yerler
            for (var k = 0; k < PopArr.length; k++) {
                PopArr[k].setMap(null);
            }

            /*
                        for (var k = 0; k < targets.length; k++) {
                            targets[k].setMap(null);
                        }
            */

            markers = [];
            circles = [];
            flightP = [];
            PopArr = [];
            //                  targets = [];

            document.getElementById("display").innerHTML = "";

            // json okunarak drone arası çizgiler çizilir
            $.getJSON('/static/json/polylines.json', function (poly) {

                var cthr = poly[2][0].cthr;

                for (var k = 0; k < poly[0].length; k++) {
                    var flightPath = new google.maps.Polyline({
                        path: [poly[0][k].drone1, poly[0][k].drone2],
                        strokeColor: "red",
                        strokeOpacity: 1.0,
                        map: map,
                        strokeWeight: 2
                    });
                    flightP.push(flightPath);

                    for (var x = 0; x < poly[1].length; x++) {

                        Ddist = Math.round(getDistance(poly[0][k].drone1, poly[1][x]) * 100) / 100;

                        if (Ddist < cthr) {

                            var flightPath = new google.maps.Polyline({
                                path: [poly[0][k].drone1, poly[1][x]],
                                strokeColor: "green",
                                strokeOpacity: 1.0,
                                map: map,
                                strokeWeight: 2
                            });
                            flightP.push(flightPath);
                        }

                        Ddist = Math.round(getDistance(poly[0][k].drone2, poly[1][x]) * 100) / 100;

                        if (Ddist < cthr) {

                            var flightPath = new google.maps.Polyline({
                                path: [poly[0][k].drone2, poly[1][x]],
                                strokeColor: "green",
                                strokeOpacity: 1.0,
                                map: map,
                                strokeWeight: 2
                            });
                            flightP.push(flightPath);
                        }
                    }
                }

                for (var p = 0; p < poly[1].length; p++) {
                    for (var q = poly[1].length - 1; q > p; q--) {

                        Ddist = Math.round(getDistance(poly[1][p], poly[1][q]) * 100) / 100;

                        if (Ddist < cthr) {
                            var flightPath = new google.maps.Polyline({
                                path: [poly[1][p], poly[1][q]],
                                strokeColor: "green",
                                strokeOpacity: 1.0,
                                map: map,
                                strokeWeight: 2
                            });
                            flightP.push(flightPath);
                        }
                    }
                }
            });

            // json okunarak drone konumları çizilir
            $.getJSON('/static/json/drone.json', function (drone) {

                for (var i = 0; i < drone.length; i++) {

                    document.getElementById("DistM" + i).remove();
                    var Newdist = document.createElement("div");
                    Newdist.id = "DistM" + i;
                    var text;
                    if (drone[i].ip == "10.0.0.1") {
                        text = "Ground " + drone[i].ip.substring(6, 8);
                    } else {
                        text = drone[i].state + " " + drone[i].ip.substring(6, 8);
                        //alert(drone[i].state);
                    }
                    var node = document.createTextNode(text);
                    Newdist.appendChild(node);
                    var element = document.getElementById("display");
                    element.appendChild(Newdist);
                    document.getElementById("DistM" + i).style.color = "black";

                    Popup = createPopupClass();
                    popup = new Popup(
                        new google.maps.LatLng(drone[i].lat, drone[i].lng),
                        document.getElementById("DistM" + i));
                    popup.setMap(map);
                    PopArr.push(popup);

                    if (drone[i].ip == "10.0.0.1") {
                        if (map_state != drone[i].lat + drone[i].lng || droneCount != drone.length) {
                            location.reload();
                        }
                    }
                    else {
                        var marker = new google.maps.Marker({
                            position: drone[i],
                            icon: Drone_icon,
                            map: map
                        });
                        markers.push(marker);
                    }
                }
            });

            // hedef konumları ve değişen stateler  çizilir
            $.getJSON('/static/json/target.json', function (tar) {

                if (tar.length != document.getElementById("dis").innerHTML) {

                    document.getElementById("dis").innerHTML = tar.length;

                    for (var k = 0; k < targets.length; k++) {
                        targets[k].setMap(null);
                    }
                    targets = [];
                    targetStates = [];

                    for (var t = 0; t < tar.length; t++) {
                        var marker = new google.maps.Marker({
                            position: tar[t],
                            icon: icons[tar[t].state],
                            map: map
                        });
                        targets.push(marker);
                        targetStates.push(marker);
                    }
                } else {

                    for (var t = 0; t < tar.length; t++) {

                        if ((tar[t].state != targetStates[t].state) & (typeof targetStates[t] !== 'undefined')) {
                            targetStates[t].state = tar[t].state;
                            targets[t].setMap(null);
                            targets[t] = new google.maps.Marker({
                                position: tar[t],
                                icon: icons[tar[t].state],
                                map: map
                            });
                        }
                    }
                }
            });
        }
    });
}

