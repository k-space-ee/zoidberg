function visualizeGamepads(values) {
    for (key in values) {
        var i = "sensor_" + key.replace(".", "_");
        var el = document.getElementById(i);
        if (!el) {
            $("#sensors").append("<div>" + key + "</div>");
            $("#sensors").append("<canvas id=\"" + i + "\" width=\"200\" height=\"50\">");
            var chart = new SmoothieChart({minValue: -1.0, maxValue: 1.0, millisPerPixel: 5});
            timeseries[key] = new TimeSeries();
            chart.addTimeSeries(timeseries[key], {
                strokeStyle: 'rgba(0, 255, 0, 1)',
                fillStyle: 'rgba(0, 255, 0, 0.2)',
                lineWidth: 2
            });
            chart.streamTo(document.getElementById(i), 0);
        }
        timeseries[key].append(new Date().getTime(), values[key]);
    }
}

var controllers = {};
var previous = "";

function streamGamepad() {
    var data = {};
    var anyMotion = 0;
    var initialRun = Object.keys(controllers).length === 0;

    var state = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var j = 0; j < state.length; j++) {
        if (!state[j]) continue;

        for (var i = 0; i < state[j].buttons.length; i++) {
            var val = state[j].buttons[i];
            var pressed = val == 1.0;
            if (typeof(val) == "object") {
                pressed = val.pressed;
                val = val.value;
            }
            var buttonKey = "controller" + state[j].index + ".button" + i;
            if (controllers[buttonKey] != pressed || pressed) {
                data[buttonKey] = pressed;
            }
            controllers[buttonKey] = pressed;
        }
        for (var i = 0; i < state[j].axes.length; i++) {
            var axisKey = "controller" + state[j].index + ".axis" + i;
            var motion = Math.round(state[j].axes[i] * 6);
            anyMotion += motion;
            if (controllers[axisKey] != state[j].axes[i] || motion) {
                data[axisKey] = state[j].axes[i];
            }
            controllers[axisKey] = state[j].axes[i];
        }
    }
    if (!initialRun){
        var newPackage = JSON.stringify({"action": "gamepad", "data": data});
        if (Object.keys(data).length || anyMotion){
            socket.send(newPackage);
            previous = newPackage;
        } else {
             socket.send(JSON.stringify({"action": "gamepad"}));
        }
    }
    // TODO: remove this when serial write speed has been fixed
    setTimeout(streamGamepad, 30);
    // requestAnimationFrame(streamGamepad);
}

function ping() {
    var package = JSON.stringify({"action": "ping"});
    socket.send(package);
    setTimeout(ping, 500);
}

function scriptSaveAs(filename) {
    $.ajax({
        url: "/api/script/" + filename,
        method: "PUT",
        dataType: "text",
        data: window.editor.getValue()
    });
}

window.timeseries = {};


function onSensorStatsReceived(values, units) {
    for (key in values) {
        var u = units ? units : {}
        var unit = key in u ? u[key] : null;
        var i = "sensor_" + key.replace(".", "_");
        var el = document.getElementById(i);

        if (!el) {
            $("#sensors").append("<div>" + key + "</div>");
            $("#sensors").append("<canvas id=\"" + i + "\" width=\"600\" height=\"100\">");
            var chart = new SmoothieChart({millisPerPixel: 50});
            timeseries[key] = new TimeSeries();
            chart.addTimeSeries(timeseries[key], {
                strokeStyle: 'rgba(0, 255, 0, 1)',
                fillStyle: 'rgba(0, 255, 0, 0.2)',
                lineWidth: 4
            });
            chart.streamTo(document.getElementById(i), 2000);
        }

        timeseries[key].append(new Date().getTime(), values[key]);
    }
}

$(document).ready(function () {

    $("#file").change(function (a, b, c) {
        $.ajax("/api/script/" + $("#file option:selected").val()).done(function (resp) {
            window.editor.setValue(resp);
            window.editor.setOption("mode", "python");
        });

    });

    $.ajax("/api/script/").done(function (resp) {
        for (var j = 0; j < resp.files.length; j++) {
            $("#file").append("<option>" + resp.files[j] + "</option>");
        }
        $("#file").trigger("change");
    });

    $("button.run").click(function () {
        socket.send(JSON.stringify({"action": "run", "filename": $("#file option:selected").val()}));
    });

    $("button.stop").click(function () {
        socket.send(JSON.stringify({"action": "stop"}));
    });

    $("button.save_as").click(function () {
        scriptSaveAs(prompt("Specify new filename"));
    });

    $("button.save").click(function () {
        scriptSaveAs($("#file option:selected").val() || prompt("Specify new filename"));
    });

    $("#toggle_recording").click(function () {
        console.info("Toggling record");
        socket.send(JSON.stringify({"action": "record_toggle"}));
    });


    window.editor = CodeMirror.fromTextArea(document.getElementById("code"), {
        lineNumbers: true,
        value: "function myScript(){return 100;}\n",
        mode: "htmlmixed"
    });

    window.editor.addKeyMap({
        "Ctrl-S": function () {
            $("button.save").click();
        },
        "F9": function () {
            console.info("Going to run");
        }
    });

    var firstLoad = true;
    socket = new ReconnectingWebSocket('ws:' + window.location.host);
    socket.onopen = function (event) {
        if (!firstLoad) {
            location.reload();
        }

        firstLoad = false;
        console.log("WebSocket opened");
        requestAnimationFrame(streamGamepad);
        setTimeout(ping, 1000);
    };

    socket.onerror = function (error) {
        console.log('WebSocket Error: ', error);
    };

    socket.onclose = function (event) {
        requestAnimationFrame(streamGamepad);
        console.log("WebSocket closed!");
    };

    socket.onmessage = function (event) {

        var msg = JSON.parse(event.data);

        switch (msg.action) {
            case "log-entry":
                // console[msg.severity].apply(this, [msg.message]);
                $("#log").prepend("<li class=\"" + msg.severity + "\">" + msg.uptime + ": <strong>" + msg.file + "</strong>:" + msg.message + "</li>");

                break
            case "sensors":
                onSensorStatsReceived(msg.values, msg.units);
                break
            case "position-robot":
                // console.log(msg);
                var imgX = 960;
                var imgY = 660;
                var y = (1 - (msg.y + 1.55) / 3.1) * imgY + 60;
                var x = (msg.x / 4.6) * imgX + 30;
                $("#marker").css("top", y);
                $("#marker").css("left", x);
                // console.log(x, y);

                break
            case "settings-packet":
                // console.log(msg.sliders, msg.options, "websocket-settings-packet");
                createInputNodes(msg);
                break;
            default:
                console.info("Unhandled message type", msg.type, "data:", msg);
        }
    }

    function setColor(values) {
        var channel = this.target.dataset.channel;
        socket.send(JSON.stringify({"action": "set_settings", [channel]: values}));
    }

    function createInputNodes(msg) {

        var settingsNode = document.getElementById("settings-be-here");
        var settings = "";

        var spawn_radio = function (index, values) {
            var name = values[0];
            var value_A = values[1][1];
            var value_B = values[1][2];
            var enabled_A = value_A === values[1][0] ? "checked" : "";
            var enabled_B = value_B === values[1][0] ? "checked" : "";
            var radio_A = "<input type='radio' name='" + name + "' value='" + value_A + "'" + enabled_A + "> " + name + " " + value_A + "<br>";
            var radio_B = "<input type='radio' name='" + name + "' value='" + value_B + "'" + enabled_B + "> " + name + " " + value_B + "<br>";
            settings += "<div class='block pull-left'><h4>" + name + "</h4><br><div>";
            settings += radio_A;
            settings += radio_B;
            if (values[1].length > 3) {
                var value_C = values[1][3];
                var enabled_C = value_C === values[1][0] ? "checked" : "";
                var radio_C = "<input type='radio' name='" + name + "' value='" + value_C + "'" + enabled_C + "> " + name + " " + value_C + "<br>";
                settings += radio_C;
            }
            settings += "</div></div>";
        };

        $.each(msg.options, spawn_radio);
        settingsNode.innerHTML = settings;

        var sliderNode = document.getElementById("sliders-be-here");
        var sliders = "";
        var sliderBlock = "";
        for (var block in msg.sliders) {
            var slider_mapping = msg.sliders[block];

            for (var group in slider_mapping) {
                var attributes = slider_mapping[group];

                var sliderGroup = "";
                var keys = "";
                for (var attribute in attributes) {
                    var range = attributes[attribute];

                    var key = block + "|" + group + "|" + attribute;

                    keys += attribute + '|';

                    sliderGroup += attribute + "<div class='slider' data-channel='" + key + "' data-min='0' data-max='255' data-start='" + range[0] + "' data-end='" + range[1] + "'></div>";
                    sliderGroup += "<br>";
                }
                keys = keys.substring(0, keys.length-1);
                sliderBlock += "<div class='col-lg-3 col-sm-6'><div class='floatBlock full-width'><h4>" + group.toUpperCase() + " " + keys + "</h4>" + sliderGroup + "</div></div>";
            }
        }

        sliders += "<div class='row'>" + sliderBlock + "</div>";
        sliderNode.innerHTML = sliders;

        sliderInit();

        $('input[type=radio]').change(function () {
            console.log("send", this.name, this.value);
            socket.send(JSON.stringify({"action": "set_options", [this.name]: this.value}));
        });
    }

    function sliderInit() {
        var sliders = $(".slider");
        // console.log('sliders', sliders);
        for (var i = 0; i < sliders.length; i++) {
            var min = parseInt(sliders[i].getAttribute("data-min"));
            var max = parseInt(sliders[i].getAttribute("data-max"));
            var start = parseInt(sliders[i].getAttribute("data-start"));
            var end = parseInt(sliders[i].getAttribute("data-end"));

            noUiSlider.create(sliders[i], {
                start: [start, end],
                behaviour: 'drag',
                animate: false,
                step: 1,
                connect: true,
                tooltips: true,
                range: {'min': min, 'max': max},
                format: {
                    to: function (value) {
                        return Math.round(value);
                    },
                    from: function (value) {
                        return value;
                    }
                }
            });
            sliders[i].noUiSlider.on('change', setColor);
        }
    }
});
