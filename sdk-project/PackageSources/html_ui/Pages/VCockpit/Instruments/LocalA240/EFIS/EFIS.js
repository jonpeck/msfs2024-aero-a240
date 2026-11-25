/*
 * Aero A240 - single-screen flight + engine display (1024 x 768 canvas).
 *
 * In the simulator this runs as a VCockpit instrument (BaseInstrument / SimVar are
 * provided by the sim). Opened in an ordinary browser it falls back to animated demo
 * data, so the layout can be worked on without starting the sim:
 *     viewer/efis.html
 */
(function () {
    "use strict";

    var W = 1024, H = 768;
    var PFD_W = 724;                 // left part: attitude, tapes, heading
    var D2R = Math.PI / 180;

    // Airspeed markings (knots)
    var VS0 = 35, VS1 = 43, VFE = 81, VNO = 108, VNE = 124;
    // Rotax 912 ULS limits
    var RPM_MAX = 5800, RPM_CAUTION = 5500;

    var IN_SIM = typeof SimVar !== "undefined" && typeof BaseInstrument !== "undefined";

    function sv(name, unit, fallback) {
        try {
            var v = SimVar.GetSimVarValue(name, unit);
            return (typeof v === "number" && isFinite(v)) ? v : fallback;
        } catch (e) {
            return fallback;
        }
    }

    function readSim() {
        return {
            // The sim reports pitch positive nose-DOWN and bank positive LEFT.
            pitch: -sv("PLANE PITCH DEGREES", "degrees", 0),
            bankLeft: sv("PLANE BANK DEGREES", "degrees", 0),
            ias: sv("AIRSPEED INDICATED", "knots", 0),
            gs: sv("GROUND VELOCITY", "knots", 0),
            alt: sv("INDICATED ALTITUDE", "feet", 0),
            baro: sv("KOHLSMAN SETTING HG", "inches of mercury", 29.92),
            vs: sv("VERTICAL SPEED", "feet per minute", 0),
            hdg: sv("PLANE HEADING DEGREES MAGNETIC", "degrees", 0),
            ball: sv("TURN COORDINATOR BALL", "position", 0),
            oat: sv("AMBIENT TEMPERATURE", "celsius", 15),
            rpm: sv("GENERAL ENG RPM:1", "rpm", 0),
            oilTemp: sv("GENERAL ENG OIL TEMPERATURE:1", "celsius", 0),
            oilPress: sv("GENERAL ENG OIL PRESSURE:1", "psi", 0),
            cht: sv("ENG CYLINDER HEAD TEMPERATURE:1", "celsius", 0),
            fuelFlow: sv("ENG FUEL FLOW GPH:1", "gallons per hour", 0),
            fuelL: sv("FUEL TANK LEFT MAIN QUANTITY", "gallons", 0),
            fuelR: sv("FUEL TANK RIGHT MAIN QUANTITY", "gallons", 0),
            volts: sv("ELECTRICAL MAIN BUS VOLTAGE", "volts", 0),
            flaps: sv("FLAPS HANDLE INDEX", "number", 0),
            trim: sv("ELEVATOR TRIM PCT", "percent", 0),
            com: sv("COM ACTIVE FREQUENCY:1", "MHz", 122.8),
            xpdr: sv("TRANSPONDER CODE:1", "number", 1200)
        };
    }

    function readDemo(t) {
        return {
            pitch: 4 + 5 * Math.sin(t * 0.5),
            bankLeft: 18 * Math.sin(t * 0.35),
            ias: 92 + 14 * Math.sin(t * 0.21),
            gs: 97,
            alt: 3480 + 220 * Math.sin(t * 0.17),
            baro: 30.02,
            vs: 420 * Math.cos(t * 0.17),
            hdg: (274 + 30 * Math.sin(t * 0.12) + 360) % 360,
            ball: 0.25 * Math.sin(t * 0.8),
            oat: 12,
            rpm: 5050 + 150 * Math.sin(t * 0.4),
            oilTemp: 96, oilPress: 52, cht: 104, fuelFlow: 4.6,
            fuelL: 8.4, fuelR: 7.9, volts: 13.8, flaps: 0, trim: 6,
            com: 122.8, xpdr: 1200
        };
    }

    // ------------------------------------------------------------------ drawing
    var FONT = "Roboto, 'Segoe UI', Arial, sans-serif";

    function text(c, s, x, y, size, color, align, bold) {
        c.font = (bold ? "bold " : "") + size + "px " + FONT;
        c.fillStyle = color || "#fff";
        c.textAlign = align || "left";
        c.textBaseline = "middle";
        c.fillText(s, x, y);
    }

    function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

    function drawAttitude(c, d) {
        var cx = PFD_W / 2, cy = 330, pxPerDeg = 9;
        c.save();
        c.beginPath(); c.rect(0, 0, PFD_W, H); c.clip();
        c.translate(cx, cy);
        c.rotate(d.bankLeft * D2R);
        c.translate(0, d.pitch * pxPerDeg);
        c.fillStyle = "#2f7fd6"; c.fillRect(-1400, -1600, 2800, 1600);      // sky
        c.fillStyle = "#8a5a2b"; c.fillRect(-1400, 0, 2800, 1600);          // ground
        c.strokeStyle = "#fff"; c.lineWidth = 3;
        c.beginPath(); c.moveTo(-1400, 0); c.lineTo(1400, 0); c.stroke();
        c.lineWidth = 2;
        for (var p = -60; p <= 60; p += 5) {
            if (p === 0) { continue; }
            var y = -p * pxPerDeg, half = (p % 10 === 0) ? 70 : 35;
            c.beginPath(); c.moveTo(-half, y); c.lineTo(half, y); c.stroke();
            if (p % 10 === 0) {
                text(c, String(Math.abs(p)), -half - 12, y, 20, "#fff", "right");
                text(c, String(Math.abs(p)), half + 12, y, 20, "#fff", "left");
            }
        }
        c.restore();

        // bank scale (fixed) and pointer (rolls with the horizon)
        c.save();
        c.translate(cx, cy);
        c.strokeStyle = "#fff"; c.lineWidth = 3;
        var marks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60], r = 235;
        for (var i = 0; i < marks.length; i++) {
            var a = (marks[i] - 90) * D2R, len = (marks[i] % 30 === 0) ? 22 : 12;
            c.beginPath();
            c.moveTo(Math.cos(a) * r, Math.sin(a) * r);
            c.lineTo(Math.cos(a) * (r + len), Math.sin(a) * (r + len));
            c.stroke();
        }
        c.rotate(d.bankLeft * D2R);
        c.fillStyle = "#fff";
        c.beginPath(); c.moveTo(0, -r); c.lineTo(-12, -r + 20); c.lineTo(12, -r + 20); c.closePath(); c.fill();
        c.restore();

        // fixed aircraft symbol
        c.save();
        c.translate(cx, cy);
        c.strokeStyle = "#000"; c.lineWidth = 9; symbol(c);
        c.strokeStyle = "#ffd400"; c.lineWidth = 5; symbol(c);
        c.restore();

        // slip ball
        c.save();
        c.translate(cx, 600);
        c.fillStyle = "rgba(0,0,0,0.55)"; c.fillRect(-90, -15, 180, 30);
        c.strokeStyle = "#fff"; c.lineWidth = 2;
        c.beginPath(); c.moveTo(-17, -15); c.lineTo(-17, 15); c.moveTo(17, -15); c.lineTo(17, 15); c.stroke();
        c.fillStyle = "#fff";
        c.beginPath(); c.arc(clamp(d.ball, -1, 1) * 72, 0, 12, 0, Math.PI * 2); c.fill();
        c.restore();
    }

    function symbol(c) {
        c.beginPath();
        c.moveTo(-150, 0); c.lineTo(-60, 0); c.lineTo(-60, 18);
        c.moveTo(150, 0); c.lineTo(60, 0); c.lineTo(60, 18);
        c.stroke();
        c.beginPath(); c.rect(-4, -4, 8, 8); c.stroke();
    }

    // Vertical moving tape. valuePerPx: units per pixel; right: draw ticks on the right edge.
    function drawTape(c, x, w, top, bottom, value, valuePerPx, minor, major, ticksRight, minValue) {
        var mid = (top + bottom) / 2;
        c.save();
        c.fillStyle = "rgba(0,0,0,0.55)"; c.fillRect(x, top, w, bottom - top);
        c.beginPath(); c.rect(x, top, w, bottom - top); c.clip();
        var lo = value - (bottom - mid) * valuePerPx, hi = value + (mid - top) * valuePerPx;
        c.strokeStyle = "#fff"; c.lineWidth = 2;
        for (var v = Math.ceil(lo / minor) * minor; v <= hi; v += minor) {
            if (v < minValue) { continue; }
            var y = mid - (v - value) / valuePerPx, isMajor = Math.abs(v % major) < 1e-6;
            var len = isMajor ? 18 : 10;
            c.beginPath();
            if (ticksRight) { c.moveTo(x + w, y); c.lineTo(x + w - len, y); }
            else { c.moveTo(x, y); c.lineTo(x + len, y); }
            c.stroke();
            if (isMajor) {
                if (ticksRight) { text(c, String(Math.round(v)), x + w - 26, y, 24, "#fff", "right"); }
                else { text(c, String(Math.round(v)), x + 26, y, 24, "#fff", "left"); }
            }
        }
        c.restore();
        return { mid: mid, yOf: function (v) { return mid - (v - value) / valuePerPx; } };
    }

    function band(c, x, w, tape, top, bottom, from, to, color) {
        var y0 = clamp(tape.yOf(to), top, bottom), y1 = clamp(tape.yOf(from), top, bottom);
        if (y1 > y0) { c.fillStyle = color; c.fillRect(x, y0, w, y1 - y0); }
    }

    function readout(c, x, y, w, h, s, size) {
        c.fillStyle = "#000"; c.fillRect(x, y - h / 2, w, h);
        c.strokeStyle = "#fff"; c.lineWidth = 2; c.strokeRect(x, y - h / 2, w, h);
        text(c, s, x + w - 10, y + 1, size, "#fff", "right", true);
    }

    function drawSpeed(c, d) {
        var x = 14, w = 112, top = 90, bottom = 570;
        var tape = drawTape(c, x, w, top, bottom, d.ias, 0.25, 5, 10, true, 0);
        var bx = x + w - 9;
        band(c, bx - 9, 9, tape, top, bottom, VS0, VFE, "#fff");
        band(c, bx, 9, tape, top, bottom, VS1, VNO, "#22c55e");
        band(c, bx, 9, tape, top, bottom, VNO, VNE, "#facc15");
        band(c, bx, 9, tape, top, bottom, VNE, 400, "#ef4444");
        readout(c, x, tape.mid, w - 20, 50, String(Math.round(Math.max(0, d.ias))), 38);
        text(c, "KT", x + w / 2, top - 22, 22, "#9fb3c8", "center");
        text(c, "GS " + Math.round(d.gs), x + w / 2, bottom + 24, 24, "#e879f9", "center");
    }

    function drawAltitude(c, d) {
        var w = 132, x = PFD_W - 14 - w - 34, top = 90, bottom = 570;
        var tape = drawTape(c, x, w, top, bottom, d.alt, 2.2, 100, 500, false, -1000);
        readout(c, x + 20, tape.mid, w - 20, 50, String(Math.round(d.alt / 10) * 10), 34);
        text(c, "FT", x + w / 2, top - 22, 22, "#9fb3c8", "center");
        text(c, d.baro.toFixed(2) + " IN", x + w / 2, bottom + 24, 24, "#22d3ee", "center");

        // vertical speed bar, +/- 1500 fpm
        var vx = x + w + 6, vw = 28, mid = tape.mid, span = (bottom - top) / 2;
        c.fillStyle = "rgba(0,0,0,0.55)"; c.fillRect(vx, top, vw, bottom - top);
        c.strokeStyle = "#fff"; c.lineWidth = 2;
        for (var f = -1500; f <= 1500; f += 500) {
            var y = mid - f / 1500 * span;
            c.beginPath(); c.moveTo(vx, y); c.lineTo(vx + (f === 0 ? vw : 10), y); c.stroke();
        }
        var vy = mid - clamp(d.vs, -1500, 1500) / 1500 * span;
        c.fillStyle = "#e879f9";
        c.fillRect(vx + 12, Math.min(mid, vy), 12, Math.abs(vy - mid));
        text(c, String(Math.round(d.vs / 10) * 10), vx + vw, d.vs >= 0 ? top - 22 : bottom + 24, 22,
             "#e879f9", "right");
    }

    function drawHeading(c, d) {
        var top = 668, h = 64, cx = PFD_W / 2, pxPerDeg = 6;
        c.save();
        c.fillStyle = "#101418"; c.fillRect(0, top, PFD_W, H - top);
        c.beginPath(); c.rect(0, top, PFD_W, h); c.clip();
        c.strokeStyle = "#fff"; c.lineWidth = 2;
        var names = { 0: "N", 90: "E", 180: "S", 270: "W" };
        for (var off = -70; off <= 70; off++) {
            var deg = Math.round(d.hdg) + off;
            if (deg % 5 !== 0) { continue; }
            var x = cx + (deg - d.hdg) * pxPerDeg, n = ((deg % 360) + 360) % 360;
            var major = n % 10 === 0;
            c.beginPath(); c.moveTo(x, top); c.lineTo(x, top + (major ? 16 : 9)); c.stroke();
            if (n % 30 === 0) {
                text(c, names[n] !== undefined ? names[n] : String(n / 10), x, top + 36, 26,
                     names[n] !== undefined ? "#facc15" : "#fff", "center", true);
            }
        }
        c.restore();
        var hd = Math.round(d.hdg) % 360;
        if (hd === 0) { hd = 360; }
        c.fillStyle = "#000"; c.fillRect(cx - 44, top - 34, 88, 36);
        c.strokeStyle = "#fff"; c.lineWidth = 2; c.strokeRect(cx - 44, top - 34, 88, 36);
        text(c, ("00" + hd).slice(-3) + "°", cx + 2, top - 15, 28, "#fff", "center", true);
    }

    // Horizontal bar gauge with coloured ranges: [[from, to, color], ...]
    function bar(c, y, label, value, shown, min, max, ranges) {
        var x = PFD_W + 22, w = W - PFD_W - 44;
        text(c, label, x, y, 21, "#9fb3c8", "left");
        text(c, shown, x + w, y, 28, "#fff", "right", true);
        var by = y + 22, bh = 12;
        c.fillStyle = "#263039"; c.fillRect(x, by, w, bh);
        var colorAtValue = "#fff";
        for (var i = 0; i < ranges.length; i++) {
            var a = clamp((ranges[i][0] - min) / (max - min), 0, 1);
            var b = clamp((ranges[i][1] - min) / (max - min), 0, 1);
            c.fillStyle = ranges[i][2]; c.fillRect(x + a * w, by + bh - 4, (b - a) * w, 4);
            if (value >= ranges[i][0] && value <= ranges[i][1]) { colorAtValue = ranges[i][2]; }
        }
        var px = x + clamp((value - min) / (max - min), 0, 1) * w;
        c.fillStyle = colorAtValue;
        c.beginPath(); c.moveTo(px, by + bh - 4); c.lineTo(px - 8, by - 6); c.lineTo(px + 8, by - 6);
        c.closePath(); c.fill();
    }

    function drawEngine(c, d) {
        var G = "#22c55e", Y = "#facc15", R = "#ef4444";
        c.fillStyle = "#0b0f13"; c.fillRect(PFD_W, 0, W - PFD_W, H);
        c.strokeStyle = "#39444f"; c.lineWidth = 2;
        c.beginPath(); c.moveTo(PFD_W, 0); c.lineTo(PFD_W, H); c.stroke();

        var x = PFD_W + 22, w = W - PFD_W - 44;
        text(c, "ROTAX 912 ULS", x, 30, 20, "#9fb3c8", "left", true);
        bar(c, 78, "RPM", d.rpm, String(Math.round(d.rpm / 10) * 10), 0, 6000,
            [[1400, RPM_CAUTION, G], [RPM_CAUTION, RPM_MAX, Y], [RPM_MAX, 6000, R]]);
        bar(c, 158, "OIL °C", d.oilTemp, String(Math.round(d.oilTemp)), 20, 140,
            [[20, 50, Y], [50, 110, G], [110, 130, Y], [130, 140, R]]);
        bar(c, 238, "OIL PSI", d.oilPress, String(Math.round(d.oilPress)), 0, 110,
            [[0, 12, R], [12, 29, Y], [29, 73, G], [73, 102, Y], [102, 110, R]]);
        bar(c, 318, "CHT °C", d.cht, String(Math.round(d.cht)), 20, 150,
            [[20, 50, Y], [50, 120, G], [120, 135, Y], [135, 150, R]]);
        bar(c, 398, "FUEL GPH", d.fuelFlow, d.fuelFlow.toFixed(1), 0, 8, [[0, 8, G]]);

        // fuel quantity, two wing tanks of 11.25 gal
        text(c, "FUEL GAL", x, 478, 21, "#9fb3c8", "left");
        var tanks = [["L", d.fuelL], ["R", d.fuelR]], tw = (w - 16) / 2;
        for (var i = 0; i < 2; i++) {
            var tx = x + i * (tw + 16), frac = clamp(tanks[i][1] / 11.25, 0, 1);
            c.fillStyle = "#263039"; c.fillRect(tx, 498, tw, 26);
            c.fillStyle = frac < 0.15 ? R : (frac < 0.3 ? Y : G); c.fillRect(tx, 498, tw * frac, 26);
            text(c, tanks[i][0] + " " + tanks[i][1].toFixed(1), tx + tw / 2, 512, 21, "#fff", "center", true);
        }
        text(c, "TOTAL " + (d.fuelL + d.fuelR).toFixed(1), x + w, 478, 21, "#fff", "right");

        var rows = [
            ["VOLTS", d.volts.toFixed(1), d.volts < 12.0 ? Y : "#fff"],
            ["FLAPS", String(Math.round(d.flaps)) + " / 2", d.flaps > 0 ? "#22d3ee" : "#fff"],
            ["TRIM", (d.trim >= 0 ? "UP " : "DN ") + Math.abs(Math.round(d.trim)) + "%", "#fff"],
            ["OAT", Math.round(d.oat) + "°C", "#fff"],
            ["COM 1", d.com.toFixed(3), G],
            ["XPDR", ("000" + Math.round(d.xpdr)).slice(-4), "#fff"]
        ];
        for (var r = 0; r < rows.length; r++) {
            var y = 566 + r * 33;
            text(c, rows[r][0], x, y, 21, "#9fb3c8", "left");
            text(c, rows[r][1], x + w, y, 25, rows[r][2], "right", true);
        }
    }

    function draw(c, d) {
        c.setTransform(1, 0, 0, 1, 0, 0);
        c.fillStyle = "#000"; c.fillRect(0, 0, W, H);
        drawAttitude(c, d);
        drawSpeed(c, d);
        drawAltitude(c, d);
        drawHeading(c, d);
        drawEngine(c, d);
    }

    // ------------------------------------------------------------------ hosting
    if (IN_SIM) {
        class LocalA240EFIS extends BaseInstrument {
            constructor() {
                super();
                this._ctx = null;
                this._lastDraw = 0;
            }
            get templateID() { return "LocalA240EFIS"; }
            get isInteractive() { return false; }
            connectedCallback() {
                super.connectedCallback();
                var self = this;
                // Safety net: if the sim stops calling Update(), keep the display alive.
                setInterval(function () {
                    if (Date.now() - self._lastDraw > 500) { self._paint(); }
                }, 250);
            }
            _paint() {
                if (!this._ctx) {
                    var canvas = this.querySelector("#efis") || document.getElementById("efis");
                    if (!canvas) { return; }
                    this._canvas = canvas;
                    this._ctx = canvas.getContext("2d");
                }
                this._lastDraw = Date.now();
                draw(this._ctx, readSim());
                // The sim's HTML renderer only refreshes the cockpit-screen texture when the page
                // changes, and drawing into a canvas does not count as a change: without this the
                // screen kept showing its first frame. Flipping an invisible style value every
                // repaint marks the page as changed. (Confirmed in the sim, 2026-11-16.)
                this._flip = !this._flip;
                this._canvas.style.opacity = this._flip ? "0.999" : "1";
            }
            Update() {
                super.Update();
                this._paint();
            }
        }
        registerInstrument("local-a240-efis", LocalA240EFIS);
    } else if (typeof document !== "undefined") {
        // Browser preview with demo data.
        window.addEventListener("load", function () {
            var canvas = document.getElementById("efis");
            if (!canvas) { return; }
            var ctx = canvas.getContext("2d"), start = Date.now();
            (function frame() {
                draw(ctx, readDemo((Date.now() - start) / 1000));
                requestAnimationFrame(frame);
            })();
        });
    }
})();
