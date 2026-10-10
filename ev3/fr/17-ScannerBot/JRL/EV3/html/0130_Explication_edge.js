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
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['691px', '37px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'intro',
                            type: 'group',
                            rect: ['203px', '-47px', '1014', '318', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['174px', '177px', '921px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Definition',
                                type: 'text',
                                rect: ['210px', '206px', '864px', '112px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"font-size: 34px;\"></span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px;\">L'utilisation de code ​à barre permet de reduire le temps de passage à la caisse,</span>Ce système permet l'enregistrement systématique des produits vendus.<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px;\"></span></p>",
                                align: "justify",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['14px', '198px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'larobotique3',
                            type: 'video',
                            tag: 'video',
                            rect: ['327px', '362px', '960', '540', 'auto', 'auto'],
                            clip: 'rect(52.80546951293945px 828.3743286132813px 496.85009765625px 130.5011444091797px)',
                            source: [vid+"explication.webm"],
                            preload: 'auto',
                            transform: [[],[],[],['1.99908','1.96675']]
                        },
                        {
                            id: 'play_boutton',
                            type: 'image',
                            rect: ['518px', '436px', '653px', '340px', 'auto', 'auto'],
                            clip: 'rect(-15.977229118347168px 510.41094970703125px 342.3923645019531px 126.45932006835938px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"play.png",'0px','0px'],
                            transform: [[],[],[],['0.39004','0.39004']]
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
                            rect: ['1256px', '847px', '46px', '47px', 'auto', 'auto'],
                            clip: 'rect(0px 60px 60px 0px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
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
                            '847px',
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
                            "eid330",
                            "scaleY",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '1.4',
                            '1.96675'
                        ],
                        [
                            "eid332",
                            "left",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '350px',
                            '327px'
                        ],
                        [
                            "eid329",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '362px',
                            '247px'
                        ],
                        [
                            "eid333",
                            "clip",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            [73.7327651977539,828.9285888671875,465.7141418457031,131.61146545410156],
                            [52.80546951293945,828.3743286132812,496.85009765625,130.5011444091797],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid331",
                            "scaleX",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '1.4',
                            '1.99908'
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
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['-55px', '-11px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(0, 61, 65); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">explication</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['-55px', '-11px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">explication</span></p>'
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
                            [0,630,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0130_Explication_edgeActions.js");
})("EDGE-6689876");
