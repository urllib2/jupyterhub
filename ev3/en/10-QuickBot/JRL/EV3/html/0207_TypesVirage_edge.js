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
                            symbolName: 'TITRE_1_1',
                            type: 'rect',
                            rect: ['499px', '25px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['262', '132px', '1064', '181', 'auto', 'auto'],
                            c: [
                            {
                                id: 'RobotR',
                                type: 'image',
                                rect: ['0px', '11px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['154px', '0px', '910px', '181px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['200px', '22px', '836px', '122px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"font-size: 36px;\">The turns are usually performed by the robot to allow it to avoid obstacles. There are different types of turns, the main ones being:​</span></p>",
                                align: "center",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none solid rgb(152, 252, 250)", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Group2',
                            type: 'group',
                            rect: ['201', '460', '1247', '184', 'auto', 'auto'],
                            c: [
                            {
                                id: 'text_block_yellowCopy',
                                type: 'image',
                                rect: ['7px', '0px', '470px', '122px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"text_block_yellow.png",'0px','0px']
                            },
                            {
                                id: 'text_block_yellow',
                                type: 'image',
                                rect: ['777px', '0px', '470px', '122px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"text_block_yellow.png",'0px','0px']
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['36px', '24px', '411px', '123px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: normal; font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif;\"><span style=\"text-decoration: underline; font-weight: 800;\">Swing </span>Turn</p>",
                                align: "center",
                                font: ['Digital-7', [146, "px"], "rgba(152,252,250,1)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text5Copy2',
                                type: 'text',
                                rect: ['807px', '18px', '400px', '123px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: normal; font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif;\"><span style=\"font-weight: 800; text-decoration: underline;\">Spin</span><span style=\"font-weight: 800;\"> </span>Turn&nbsp;</p>",
                                align: "center",
                                font: ['Digital-7', [146, "px"], "rgba(152,252,250,1)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'exemple2',
                            type: 'group',
                            rect: ['417px', '448px', '798', '375', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['513px', '-95px', '543px', '423px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(81, 0, 236);\">The Spin turn</span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-weight: 500; color: rgb(0, 0, 0);\"></span></p><p style=\"margin: 0px; text-align: left;\">​</p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-weight: 500; color: rgb(0, 0, 0);\"></span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-weight: 500; color: rgb(0, 0, 0); font-size: 36px;\">One wheel moves forward while the other reverses.</span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-size: 36px;\">​</span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-weight: 500; color: rgb(0, 0, 0);\"></span></p>",
                                align: "center",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(152,252,250,1)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'virage_serr',
                                type: 'video',
                                tag: 'video',
                                rect: ['-1545px', '-95px', '3188px', '449px', 'auto', 'auto'],
                                clip: 'rect(0px 2045.333984375px 449px 1239.9990234375px)',
                                loop: 'loop',
                                source: [vid+"virage%20serr%C3%A9.webm"],
                                preload: 'auto'
                            }]
                        },
                        {
                            id: 'exemple1',
                            type: 'group',
                            rect: ['-1035px', '335px', '3193', '449', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['2005px', '18px', '529px', '282px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(81, 0, 236);\">The swing turn</span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(0, 0, 0); font-weight: 400;\"></span></p><p style=\"margin: 0px; text-align: left;\">​</p><p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(0, 0, 0); font-weight: 400;\"></span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-weight: 500; color: rgb(0, 0, 0); font-size: 36px;\">One wheel moves forward or backward, while the other does not turn</span></p><p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(0, 0, 0); font-weight: 400;\"></span></p><p style=\"margin: 0px; text-align: left;\">.<br></p><p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(0, 0, 0); font-weight: 400;\"></span></p>",
                                align: "center",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(152,252,250,1)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'virage_large_',
                                type: 'video',
                                tag: 'video',
                                rect: ['-85px', '13px', '3227px', '454px', 'auto', 'auto'],
                                clip: 'rect(0px 3227px 454px 1233.3330078125px)',
                                source: [vid+"virage%20large.webm"],
                                preload: 'auto'
                            }]
                        },
                        {
                            id: 'boutonCopy',
                            type: 'group',
                            rect: ['213px', '832px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1060.98779296875px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGECopy',
                                type: 'image',
                                rect: ['910px', '7px', '151px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE4.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy',
                                type: 'image',
                                rect: ['910px', '7px', '151px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY4.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy',
                                type: 'text',
                                rect: ['907px', '17px', '163px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px; text-align: center;\">​Demo 2</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['-497px', '832px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1068.39501953125px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['910px', '7px', '151px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE4.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['910px', '7px', '151px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY4.png",'0px','0px']
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['914px', '15px', '145px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px; text-align: center;\">Demo 1</p><p style=\"margin: 0px; text-align: center;\">​</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
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
                            rect: ['-22px', '-3px', '655px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Turn types</p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['MS Serif, New York, serif', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['139px', '-3px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 680.7080078125px 57px -78.28271484375px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">turn types</p>'
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
                            [0,680.7080078125,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0207_TypesVirage_edgeActions.js");
})("EDGE-6689876");
