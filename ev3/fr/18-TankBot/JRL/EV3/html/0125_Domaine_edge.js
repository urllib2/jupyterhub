/*jslint */
/*global AdobeEdge: false, window: false, document: false, console:false, alert: false */
(function (compId) {

    "use strict";
    var im='images/',
        aud='media/',
        vid='media/',
        js='js/',
        fonts = {
            'Digital-7': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            '\'Digital tech\'': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            'MagistralC': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />'        },
        opts = {
            'gAudioPreloadPreference': 'auto',
            'gVideoPreloadPreference': 'auto'
        },
        resources = [
        ],
        scripts = [
            js+"jquery-2.0.3.min.js",
            js+"jquery.ui.touch-punch.min.js",
            js+"jquery-ui.min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "height",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'Oz_test',
                            type: 'video',
                            tag: 'video',
                            rect: ['326px', '238px', '960', '540', 'auto', 'auto'],
                            clip: 'rect(73.80748748779297px 831.0726318359375px 467.72869873046875px 130.97494506835938px)',
                            source: [vid+"domaine.webm"],
                            preload: 'auto',
                            transform: [[],[],[],['1.96408','1.88004']]
                        },
                        {
                            id: 'play_boutton',
                            type: 'image',
                            rect: ['503px', '232px', '653px', '340px', 'auto', 'auto'],
                            clip: 'rect(5.5556640625px 496.05517578125px 340px 152.77734375px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"play.png",'0px','0px'],
                            transform: [[],[],[],['0.39457','0.39457']]
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['80px', '878px', '1450px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre2.png",'0px','0px']
                        },
                        {
                            id: 'full_screen',
                            type: 'image',
                            rect: ['1226px', '594px', '46px', '47px', 'auto', 'auto'],
                            clip: 'rect(0px 60px 60px 0px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px'],
                            transform: [[],[],[],['1','0.98077']]
                        },
                        {
                            id: 'car1',
                            type: 'image',
                            rect: ['610px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"c2.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'car',
                            type: 'image',
                            rect: ['610px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"c1.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'tractor1',
                            type: 'image',
                            rect: ['810px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"d1.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'tractor',
                            type: 'image',
                            rect: ['810px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"d2.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'research1',
                            type: 'image',
                            rect: ['1010px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b2.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'research',
                            type: 'image',
                            rect: ['1010px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"b1.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'camera1',
                            type: 'image',
                            rect: ['410px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"e2.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'camera',
                            type: 'image',
                            rect: ['410px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"e1.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'Text5',
                            type: 'text',
                            rect: ['432px', '701px', '172px', '28px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; line-height: 32px;\"><span style=\"font-size: 28px; font-family: Verdana, Geneva, sans-serif; font-weight: 700; word-spacing: 0px; color: rgb(0, 0, 0);\">Camera</span></p>",
                            font: ['Arial, Helvetica, sans-serif', [13, "px"], "rgba(37,154,224,1.00)", "normal", "none", "", "break-word", "normal"],
                            textStyle: ["", "41px", "", "", ""]
                        },
                        {
                            id: 'Text1',
                            type: 'text',
                            rect: ['631px', '701px', '172px', '28px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; line-height: 32px;\"><span style=\"font-size: 28px; font-family: Verdana, Geneva, sans-serif; font-weight: 700; word-spacing: 0px; color: rgb(0, 0, 0);\">Télephone</span></p>",
                            font: ['Arial, Helvetica, sans-serif', [13, "px"], "rgba(37,154,224,1.00)", "normal", "none", "", "break-word", "normal"],
                            textStyle: ["", "41px", "", "", ""]
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['823px', '701px', '181px', '28px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; line-height: 32px; text-align: center;\"><span style=\"font-size: 28px; font-family: Verdana, Geneva, sans-serif; font-weight: 700; word-spacing: 0px; color: rgb(0, 0, 0);\">Bateau</span></p><p style=\"margin: 0px; line-height: 32px; text-align: center;\"><span style=\"font-size: 28px; font-family: Verdana, Geneva, sans-serif; font-weight: 700; word-spacing: 0px; color: rgb(0, 0, 0);\">​</span></p>",
                            font: ['Arial, Helvetica, sans-serif', [13, "px"], "rgba(37,154,224,1.00)", "normal", "none", "", "break-word", "normal"],
                            textStyle: ["", "41px", "", "", ""]
                        },
                        {
                            id: 'Text3',
                            type: 'text',
                            rect: ['1030px', '701px', '172px', '28px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; line-height: 32px;\"><span style=\"font-size: 28px; font-family: Verdana, Geneva, sans-serif; font-weight: 700; word-spacing: 0px; color: rgb(0, 0, 0);\">Jeux video</span></p>",
                            font: ['Arial, Helvetica, sans-serif', [13, "px"], "rgba(37,154,224,1.00)", "normal", "none", "", "break-word", "normal"],
                            textStyle: ["", "41px", "", "", ""]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['539px', '27px', '390', '57', 'auto', 'auto']
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1080', 'auto', 'auto'],
                            overflow: 'hidden',
                            fill: ["rgba(240,240,240,0)"]
                        }
                    }
                },
                timeline: {
                    duration: 6000,
                    autoPlay: false,
                    data: [
                        [
                            "eid224",
                            "left",
                            5000,
                            1000,
                            "linear",
                            "${full_screen}",
                            '1226px',
                            '1426px'
                        ],
                        [
                            "eid223",
                            "height",
                            5000,
                            1000,
                            "linear",
                            "${full_screen}",
                            '52px',
                            '57px'
                        ],
                        [
                            "eid222",
                            "top",
                            5000,
                            1000,
                            "linear",
                            "${full_screen}",
                            '594px',
                            '812px'
                        ],
                        [
                            "eid225",
                            "width",
                            5000,
                            1000,
                            "linear",
                            "${full_screen}",
                            '51px',
                            '56px'
                        ],
                        [
                            "eid266",
                            "scaleY",
                            5000,
                            1000,
                            "linear",
                            "${Oz_test}",
                            '1.34158',
                            '1.88004'
                        ],
                        [
                            "eid270",
                            "clip",
                            5000,
                            1000,
                            "linear",
                            "${Oz_test}",
                            [73.80748748779297,831.0726318359375,467.72869873046875,130.97494506835938],
                            [48.4642333984375,834.05419921875,515.8063354492188,125.75720977783203],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid267",
                            "scaleX",
                            5000,
                            1000,
                            "linear",
                            "${Oz_test}",
                            '1.34158',
                            '1.96408'
                        ],
                        [
                            "eid264",
                            "top",
                            5000,
                            1000,
                            "linear",
                            "${Oz_test}",
                            '113px',
                            '238px'
                        ],
                        [
                            "eid268",
                            "left",
                            5000,
                            1000,
                            "linear",
                            "${Oz_test}",
                            '336px',
                            '326px'
                        ]
                    ]
                }
            },
            "TITRE_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-67px', '0px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Domaines d</span><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-weight: 500;\">\'</span><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">application</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-57px', '0px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Domaines d</span><span style=\"font-size: 62px; font-weight: 500;\">\'</span><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">application</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 687.74853515625px 57px -78.28271484375px)',
                            id: 'Text6'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,505,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,57,0],
                            [0,530,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,0],
                            [0,540,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,540,57,0],
                            [0,565,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565,57,0],
                            [0,595,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid205",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,0],
                            [0,687.74853515625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0125_Domaine_edgeActions.js");
})("EDGE-6689876");
