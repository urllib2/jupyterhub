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
                            id: 'intro',
                            type: 'group',
                            rect: ['261px', '-44px', '1014', '318', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['153px', '157px', '880px', '170px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_RED1.png",'0px','0px']
                            },
                            {
                                id: 'Definition',
                                type: 'text',
                                rect: ['192px', '176px', '836px', '112px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​A player is seated at each end of a board. According to the push of his feet on the ground, the board makes the players go up and down.<span style=\"font-size: 34px;\"></span></p><p style=\"margin:0px\"><span style=\"font-size: 34px;\"></span></p>",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                            }]
                        },
                        {
                            id: 'ROBOT06',
                            type: 'image',
                            rect: ['246px', '122px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotW.png",'0px','0px']
                        },
                        {
                            id: 'larobotique3',
                            type: 'video',
                            tag: 'video',
                            rect: ['-130px', '50px', '1920', '1080', 'auto', 'auto'],
                            clip: 'rect(143.49473571777344px 1671.1893310546875px 939.9998779296875px 251.25433349609375px)',
                            source: [vid+"larobotique%284%29.webm"],
                            preload: 'auto',
                            transform: [[],[],[],['0.7','0.7']]
                        },
                        {
                            id: 'play_boutton',
                            type: 'image',
                            rect: ['520px', '407px', '653px', '340px', 'auto', 'auto'],
                            clip: 'rect(-15.977229118347168px 510.41094970703125px 342.3923645019531px 126.45932006835938px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"play.png",'0px','0px'],
                            transform: [[],[],[],['0.47059','0.47059']]
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['80px', '878px', '1450px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                        },
                        {
                            id: 'full_screen',
                            type: 'image',
                            rect: ['1256px', '794px', '46px', '47px', 'auto', 'auto'],
                            clip: 'rect(0px 60px 60px 0px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1_1',
                            type: 'rect',
                            rect: ['529px', '30px', '390', '57', 'auto', 'auto']
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
                    duration: 5500,
                    autoPlay: false,
                    data: [
                        [
                            "eid213",
                            "left",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '1256px',
                            '1432px'
                        ],
                        [
                            "eid211",
                            "height",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '52px',
                            '57px'
                        ],
                        [
                            "eid212",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '794px',
                            '815px'
                        ],
                        [
                            "eid214",
                            "width",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '51px',
                            '56px'
                        ],
                        [
                            "eid331",
                            "scaleY",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '0.7',
                            '0.99249'
                        ],
                        [
                            "eid332",
                            "scaleX",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '0.7',
                            '0.99'
                        ],
                        [
                            "eid330",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '50px',
                            '-28px'
                        ],
                        [
                            "eid334",
                            "clip",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            [143.49473571777344,1671.1893310546875,939.9998779296875,251.25433349609375],
                            [117.80679321289062,1665.0965576171875,990.529541015625,257.8533630371094],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid333",
                            "left",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '-130px',
                            '-155px'
                        ]
                    ]
                }
            },
            "TITRE_1_1": {
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
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['-56px', '-1px', '655px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 657.021728515625px 70.265380859375px 0px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px;\"><span style=\"font-weight: normal;\">​</span><span style=\"font-weight: 500;\">see Saw</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['MS Serif, New York, serif', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['-78px', '-1px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 680.7080078125px 67.2041015625px -78.28271484375px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\"><span style=\"font-weight: 500;\">See Saw</span></p>'
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
                            "eid118",
                            "clip",
                            0,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,68.224609375,0],
                            [0,190,66.18359375,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,66.18359375,0],
                            [0,220,67.2041015625,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,67.2041015625,0],
                            [0,260,66.183837890625,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,66.183837890625,0],
                            [0,270,65.163330078125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,65.163330078125,0],
                            [0,309.3876953125,65.163330078125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,309.3876953125,65.163330078125,0],
                            [0,342.44873046875,65.1630859375,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,342.44873046875,65.1630859375,0],
                            [0,380.6123046875,64.142822265625,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380.6123046875,64.142822265625,0],
                            [0,415.71435546875,67.2041015625,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,415.71435546875,67.2041015625,0],
                            [0,425.40771484375,68.224609375,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,425.40771484375,68.224609375,0],
                            [0,463.775390625,65.1630859375,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,463.775390625,65.1630859375,0],
                            [0,494.7958984375,66.183837890625,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,494.7958984375,66.183837890625,0],
                            [0,530,72.30615234375,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,72.30615234375,0],
                            [0,680.7080078125,67.2041015625,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0150_PrincipeBaloncoire_edgeActions.js");
})("EDGE-6689876");
