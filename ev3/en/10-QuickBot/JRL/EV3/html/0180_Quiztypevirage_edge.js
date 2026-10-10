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
                            id: 'Group',
                            type: 'group',
                            rect: ['324px', '98px', '872', '160', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['0px', '0px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_ORANGE',
                                type: 'image',
                                rect: ['143px', '26px', '743px', '127px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['176px', '45px', '697px', '82px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Watch the video then check the type of turn to perform in this mission<span style=\"font-size: 34px;\"></span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Question1',
                            type: 'group',
                            rect: ['143px', '602px', '340', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'E_1',
                                type: 'group',
                                rect: ['444px', '108px', '80', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV1',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR1',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG1',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix1',
                                    type: 'image',
                                    rect: ['18px', '14px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            },
                            {
                                id: 'E_0',
                                type: 'group',
                                rect: ['130px', '108px', '80', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV0',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR0',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG0',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix0',
                                    type: 'image',
                                    rect: ['18px', '14px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            },
                            {
                                id: 'E_2',
                                type: 'group',
                                rect: ['778px', '108px', '80', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV2',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR2',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG2',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix2',
                                    type: 'image',
                                    rect: ['18px', '14px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            }]
                        },
                        {
                            id: 'Question2',
                            type: 'group',
                            rect: ['1184px', '656px', '340', '67px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'E_3',
                                type: 'group',
                                rect: ['46px', '54px', '80', '67px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV3',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR3',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG3',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix3',
                                    type: 'image',
                                    rect: ['18px', '14px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            }]
                        },
                        {
                            id: 'Attention',
                            type: 'text',
                            rect: ['610px', '837px', '441px', '29px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 40px;\">Check an answer !!</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [37, "px"], "rgba(96,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'resultat',
                            type: 'group',
                            rect: ['561px', '801px', '1025', '111', 'auto', 'auto'],
                            clip: 'rect(0px 976.2822265625px 110px 0px)',
                            c: [
                            {
                                id: 'message',
                                type: 'text',
                                rect: ['55px', '28px', '1025px', '83px', 'auto', 'auto'],
                                clip: 'rect(0px 872.918212890625px 83px 0px)',
                                text: "<p style=\"margin: 0px;\">​<span style=\"color: rgb(205, 135, 0); font-size: 53px;\">message</span></p>",
                                font: ['MagistralC', [51, "px"], "rgba(63,93,255,1.00)", "600", "none", "", "break-word", "normal"],
                                transform: [[],[],[],['1.10575']]
                            }]
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['441px', '795px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['926px', '17px', '111px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​Validate</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'quiz_frein',
                            type: 'video',
                            tag: 'video',
                            rect: ['427px', '199px', '795px', '447px', 'auto', 'auto'],
                            clip: 'rect(61.33349609375px 687px 385.333251953125px 110.6669921875px)',
                            source: [vid+"quick.webm"],
                            preload: 'auto'
                        },
                        {
                            id: 'play_boutton',
                            type: 'image',
                            rect: ['504px', '253px', '653px', '340px', 'auto', 'auto'],
                            clip: 'rect(-15.977229118347168px 510.41094970703125px 342.3923645019531px 126.45932006835938px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"play2.png",'0px','0px'],
                            transform: [[],[],[],['0.24412','0.24412']]
                        },
                        {
                            id: 'star_bot_op',
                            type: 'image',
                            rect: ['1338px', '119px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot1',
                            type: 'image',
                            rect: ['1338px', '119px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'Group2',
                            type: 'group',
                            rect: ['181', '599', '1256', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['0px', '0px', '279px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Swing Turn 180 degrees</p>",
                                align: "center",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['372px', '0px', '239px', '75px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: center;\"><span style=\"color: rgb(0, 0, 0);\">Spin turn&nbsp;</span></p><p style=\"margin: 0px; text-align: center;\"><span style=\"color: rgb(0, 0, 0);\">​180 degrees</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['691px', '0px', '239px', '75px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: center;\">​<span style=\"color: rgb(0, 0, 0);\">Spin turn</span></p><p style=\"margin: 0px; text-align: center;\"><span style=\"color: rgb(0, 0, 0);\">​90 degrees</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text6',
                                type: 'text',
                                rect: ['1004px', '0px', '252px', '75px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: center;\">​<span style=\"color: rgb(0, 0, 0);\">Swing turn&nbsp;</span></p><p style=\"margin: 0px; text-align: center;\"><span style=\"color: rgb(0, 0, 0);\">​90 degrees</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
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
                    duration: 5250,
                    autoPlay: false,
                    data: [

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
                            id: 'Text6Copy2',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Quiz</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-62px', '0px', '655px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            align: 'center'
                        },
                        {
                            type: 'text',
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Quiz</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['208px', '-1px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
                            align: 'center'
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
                            [0,519.83837890625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0180_Quiztypevirage_edgeActions.js");
})("EDGE-6689876");
