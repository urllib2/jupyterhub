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
                scaleToFit: "both",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['85px', '0px', '1440px', '1080px', 'auto', 'auto'],
                            clip: 'rect(0px 1440px 1080px 0px)',
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px'],
                            transform: [[],[],[],['1.00694']]
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['262px', '105px', '1128px', '177', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['121px', '22px', '900px', '250px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['165px', '14px', '820px', '157px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-weight: 800;\"></span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​According to the previous activities, we can turn the large motor in different modes </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">(rotation, second and degree)</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> a</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(199, 125, 4); font-size: 37px;\"> </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(199, 125, 4); font-size: 37px; text-decoration: underline;\">positive value</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> of the power turns the motor in a clockwise direction</span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 32px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 32px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\"></span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT07',
                                type: 'image',
                                rect: ['-25px', '49px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['692px', '29px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['80px', '878px', '1449px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                        },
                        {
                            id: 'positif',
                            type: 'group',
                            rect: ['360px', '441px', '889', '286', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A2_Clock',
                                type: 'image',
                                rect: ['724px', '41px', '192px', '189px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_Clock.gif",'0px','0px']
                            },
                            {
                                id: 'A2_motor_simple3',
                                type: 'image',
                                rect: ['-108px', '-53px', '720px', '350px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_motor%20simple.gif",'0px','0px'],
                                transform: [[],[],[],['0.7','0.7']]
                            },
                            {
                                id: 'a2_flecheclock',
                                type: 'image',
                                rect: ['524px', '111px', '99px', '82px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"a2_flecheclock.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['457px', '36px', '449px', '285px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-family: MagistralC; font-size: 43px;\">Clockwise</span></p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(236,150,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'question',
                            type: 'group',
                            rect: ['370px', '170px', '769px', '120', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Hint_green',
                                type: 'image',
                                rect: ['0px', '-58px', '889px', '186px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px']
                            },
                            {
                                id: 'Text6',
                                type: 'text',
                                rect: ['61px', '2px', '805px', '115px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px;\">How do you run the motor counterclockwise?​</span></p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'negatif',
                                type: 'group',
                                rect: ['-37px', '167px', '889', '286', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'A2_ClockCopy',
                                    type: 'image',
                                    rect: ['734px', '48px', '192px', '189px', 'auto', 'auto'],
                                    clip: 'rect(0px 164.97314453125px 189px 24.32421875px)',
                                    fill: ["rgba(0,0,0,0)",im+"A2_Clock2.gif",'0px','0px'],
                                    transform: [[],[],[],['1.31788','0.99206']]
                                },
                                {
                                    id: 'A2_motor_simple3Copy',
                                    type: 'image',
                                    rect: ['-108px', '-53px', '720px', '350px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"A2_motor%20simple2.gif",'0px','0px'],
                                    transform: [[],[],[],['0.7','0.7']]
                                },
                                {
                                    id: 'a2_flecheclockCopy',
                                    type: 'image',
                                    rect: ['526px', '81px', '99px', '82px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"a2_flecheclock2.png",'0px','0px']
                                },
                                {
                                    id: 'TextCopy',
                                    type: 'text',
                                    rect: ['377px', '-21px', '449px', '285px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: MagistralC; font-size: 43px; color: rgb(9, 21, 136);\">CounterClockWise</span></p>",
                                    align: "left",
                                    font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(236,150,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            }]
                        },
                        {
                            id: 'A2_puissnagatif',
                            type: 'group',
                            rect: ['247px', '588px', '1049', '277px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A2_puissnagatif2',
                                type: 'image',
                                rect: ['830px', '-3px', '267px', '316px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_puissnagatif.gif",'0px','0px']
                            },
                            {
                                id: 'ExplicationCopy',
                                type: 'text',
                                rect: ['81px', '5px', '729px', '29px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(10, 146, 40);\">To turn the large motor counterclockwise, choose a </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">negative</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(10, 146, 40);\"> power value</span></p>",
                                align: "left",
                                font: ['\'Lucida Sans Unicode\', \'Lucida Grande\', sans-serif', [40, "px"], "rgba(199,125,4,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'demo',
                            type: 'group',
                            rect: ['-226px', '792px', '1051px', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1054.28564453125px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['1.22558','1.13858']],
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
                                rect: ['921px', '19px', '113px', '32px', 'auto', 'auto'],
                                clip: 'rect(5.479248046875px 116.58056640625px 45.478271484375px -2.97216796875px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 26px;\">Question</span></p>",
                                align: "left",
                                font: ['MagistralC', [0, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['941px', '19px', '113px', '32px', 'auto', 'auto'],
                                clip: 'rect(-11.32763671875px 118.26123046875px 45.478271484375px -7.173828125px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 26px;\">Reply</span></p>",
                                align: "left",
                                font: ['MagistralC', [0, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
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
                    duration: 5000,
                    autoPlay: false,
                    data: [
                        [
                            "eid478",
                            "left",
                            0,
                            0,
                            "linear",
                            "${TITRE}",
                            '692px',
                            '692px'
                        ]
                    ]
                }
            },
            "TITRE": {
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
                            font: ['Digital-7', [117, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['-4px', '0px', '859px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 900px 57px -196.29614257813px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Activity</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['-4px', '-1px', '766px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 650px 57px -155.55541992188px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Activity</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5000,
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
                            [0,515,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,515,57,0],
                            [0,530,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid173",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,-155.555419921875],
                            [0,555,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid175",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,555,57,-155.555419921875],
                            [0,595,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid177",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,-155.555419921875],
                            [0,650,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid476",
                            "top",
                            5000,
                            0,
                            "linear",
                            "${Text6}",
                            '-1px',
                            '-1px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0185_SensAntiHorraire_edgeActions.js");
})("EDGE-6689876");
