
function visualizeGamepads(values) {
    for (key in values) {
        var i = "sensor_" + key.replace(".", "_");
        var el = document.getElementById(i);
        if (!el) {
            $("#sensors").append("<div>" + key + "</div>");
            $("#sensors").append("<canvas id=\"" + i + "\" width=\"200\" height=\"50\">");
            var chart = new SmoothieChart({minValue:-1.0,maxValue:1.0, millisPerPixel:5});
            timeseries[key] = new TimeSeries();
            chart.addTimeSeries(timeseries[key], { strokeStyle: 'rgba(0, 255, 0, 1)', fillStyle: 'rgba(0, 255, 0, 0.2)', lineWidth: 2 });
            chart.streamTo(document.getElementById(i), 0);
        }
        timeseries[key].append(new Date().getTime(), values[key]);
    }
}

function streamGamepad() {
  var data = {};
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
          if (controllers["controller" + state[j].index + ".button" + i] != pressed) {
              data["controller" + state[j].index + ".button" + i] = pressed;
          }
          controllers["controller" + state[j].index + ".button" + i] = pressed;
      }
      for (var i = 0; i < state[j].axes.length; i++) {
          if (controllers["controller" + state[j].index + ".axis" + i] != state[j].axes[i]) {
              data["controller" + state[j].index + ".axis" + i] = state[j].axes[i];
          }
          controllers["controller" + state[j].index + ".axis" + i] = state[j].axes[i];
      }
  }



  if (socket.readyState == 1) {
    // WTF?!
    for (j in data) {
      console.info("Sending:", data);
      socket.send(JSON.stringify({"action": "gamepad", "data": data}));
      break;
    }
    // visualizeGamepads(controllers);
    requestAnimationFrame(streamGamepad);
  }
}

function scriptSaveAs(filename) {
    $.ajax({
        url: "/api/script/" + filename,
        method:"PUT",
        dataType:"text",
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
            var chart = new SmoothieChart({millisPerPixel:50});
            timeseries[key] = new TimeSeries();
            chart.addTimeSeries(timeseries[key], { strokeStyle: 'rgba(0, 255, 0, 1)', fillStyle: 'rgba(0, 255, 0, 0.2)', lineWidth: 4 });
            chart.streamTo(document.getElementById(i), 2000);
        }

        timeseries[key].append(new Date().getTime(), values[key]);
    }
}

$(document).ready(function() {

    $("#file").change(function(a,b,c) {
        $.ajax("/api/script/" + $("#file option:selected").val()).done(function(resp) {
            window.editor.setValue(resp);
            window.editor.setOption("mode", "python");
        });

    });

    $.ajax("/api/script/").done(function(resp) {
        for (var j = 0; j < resp.files.length; j++) {
            $("#file").append("<option>" + resp.files[j] + "</option>");
        }
        $("#file").trigger("change");
    });

    $("button.run").click(function() {
        socket.send(JSON.stringify({"action":"run", "filename":$("#file option:selected").val()}));
    });

    $("button.stop").click(function() {
        socket.send(JSON.stringify({"action":"stop"}));
    });

    $("button.save_as").click(function() {
        scriptSaveAs(prompt("Specify new filename"));
    });

    $("button.save").click(function() {
        scriptSaveAs($("#file option:selected").val() || prompt("Specify new filename"));
    });

    $("#toggle_recording").click(function() {
        console.info("Toggling record");
        socket.send(JSON.stringify({"action":"record_toggle"}));
    });


    window.editor = CodeMirror.fromTextArea(document.getElementById("code"), {
        lineNumbers: true,
        value: "function myScript(){return 100;}\n",
        mode:  "htmlmixed"
    });

    window.editor.addKeyMap({
        "Ctrl-S": function() { $("button.save").click();},
        "F9": function() { console.info("Going to run"); }
    });


    socket = new ReconnectingWebSocket('ws:'+window.location.host);
    socket.onopen = function (event) {
        console.log("WebSocket opened");
        window.controllers = {};
        requestAnimationFrame(streamGamepad);
    };

    socket.onerror = function (error) {
        console.log('WebSocket Error: ', error);
    };

    socket.onclose = function (event) {
        requestAnimationFrame(streamGamepad);
        console.log("WebSocket closed!");
    };

    socket.onmessage = function(event){

        var msg = JSON.parse(event.data);

        switch(msg.action) {
            case "log-entry":
                console[msg.severity].apply(this, [msg.message]);
                $("#log").prepend("<li class=\"" + msg.severity + "\">" + msg.uptime + ": " + msg.message + "</li>");

                break
            case "sensors":
                onSensorStatsReceived(msg.values, msg.units);
                break
            case "position-robot":
                console.log(msg);
                var imgX = 960;
                var imgY = 660;
                var y = (1 - (msg.y + 1.55) / 3.1) * imgY + 60;
                var x = (msg.x /  4.6) * imgX + 30;
                $( "#marker" ).css( "top", y);
                $( "#marker" ).css( "left", x);
                console.log(x, y);

                break
            default:
                console.info("Unhandled message type", msg.type, "data:", msg);
        }
    }
});
