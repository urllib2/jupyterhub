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
                            id: 'dr_floucar2',
                            type: 'image',
                            rect: ['51px', '367px', '1490px', '772px', 'auto', 'auto'],
                            clip: 'rect(115.78955078125px 1431.10498046875px 489.4208984375px 63.15771484375px)',
                            fill: ["rgba(0,0,0,0)",im+"dr_floucar2.png",'0px','0px']
                        },
                        {
                            id: 'motor',
                            type: 'group',
                            rect: ['364px', '154px', '475', '641', 'auto', 'auto'],
                            c: [
                            {
                                id: 'dr_motor2',
                                type: 'image',
                                rect: ['54px', '453px', '171px', '188px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"dr_motor2.png",'0px','0px']
                            },
                            {
                                id: 'Hint_green',
                                type: 'image',
                                rect: ['-61px', '12px', '872px', '656px', 'auto', 'auto'],
                                clip: 'rect(11.4287109375px 872px 656px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px'],
                                transform: [[],[],[],['0.65596','0.42154']]
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['140px', '234px', '475px', '138px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left; line-height: 44px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 33px; font-weight: 400; color: rgb(0, 0, 0);\">A powerful engine is one of the most important things, even a fighter jet engine can be used.​​​</span></p>",
                                align: "center",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [44, "px"], "rgba(1,66,65,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "41px", "", "none"]
                            }]
                        },
                        {
                            id: 'frame',
                            type: 'group',
                            rect: ['44px', '307px', '1503', '831', 'auto', 'auto'],
                            c: [
                            {
                                id: 'dr_frame3',
                                type: 'image',
                                rect: ['0px', '56px', '1503px', '779px', 'auto', 'auto'],
                                clip: 'rect(297.142822265625px 1437.28564453125px 493.28564453125px 71.42822265625px)',
                                fill: ["rgba(0,0,0,0)",im+"dr_frame3.png",'0px','0px']
                            },
                            {
                                id: 'Hint_green2',
                                type: 'image',
                                rect: ['781px', '146px', '572px', '241px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px']
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['829px', '187px', '475px', '154px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left; line-height: 41px;\">​<span style=\"color: rgb(0, 0, 0); font-weight: 400;\">​</span><span style=\"color: rgb(0, 0, 0); font-weight: 400; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span><span style=\"color: rgb(0, 0, 0); font-weight: 400; font-size: 30px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span><span style=\"color: rgb(0, 0, 0); font-weight: 400; font-size: 33px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​​Dragsters have to be so long to increase stability at high speeds.</span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 39px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 36px;\"></span></p>",
                                align: "center",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [30, "px"], "rgba(1,66,65,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'wheels',
                            type: 'group',
                            rect: ['232', '361px', '531', '477', 'auto', 'auto'],
                            c: [
                            {
                                id: 'dr_wheels2',
                                type: 'image',
                                rect: ['0px', '307px', '212px', '170px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"dr_wheels2.png",'0px','0px']
                            },
                            {
                                id: 'Hint_green3',
                                type: 'image',
                                rect: ['50px', '0px', '514px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px']
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['100px', '12px', '419px', '160px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 39px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 36px;\">​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px;\">The wheels at the rear of the car help the engine accelerate the dragster.</span><span style=\"font-size: 36px;\">&nbsp;</span></p>",
                                align: "center",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [33, "px"], "rgba(1,66,65,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'wings',
                            type: 'group',
                            rect: ['120', '220px', '500', '552', 'auto', 'auto'],
                            c: [
                            {
                                id: 'dr_wing3',
                                type: 'image',
                                rect: ['0px', '291px', '323px', '261px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"dr_wing3.png",'0px','0px']
                            },
                            {
                                id: 'Hint_green4',
                                type: 'image',
                                rect: ['181px', '140px', '552px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px']
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['218px', '189px', '478px', '96px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 39px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 36px;\"></span></p><p style=\"margin: 0px; text-align: left; line-height: 41px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 33px; font-weight: 400; color: rgb(0, 0, 0);\">The spoilers lower the rear of the car to improve its stability.​​​</span></p>",
                                align: "center",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [33, "px"], "rgba(1,66,65,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "33px", "", "none"]
                            }]
                        },
                        {
                            id: 'dr_wheels2Copy',
                            type: 'image',
                            rect: ['232px', '668px', '212px', '170px', 'auto', 'auto'],
                            clip: 'rect(0px 192px 170px 0px)',
                            opacity: '0',
                            fill: ["rgba(0,0,0,0)",im+"dr_wheels2.png",'0px','0px']
                        },
                        {
                            id: 'dr_frame3Copy',
                            type: 'image',
                            rect: ['17px', '347px', '1503px', '779px', 'auto', 'auto'],
                            clip: 'rect(297.142822265625px 1437.28564453125px 493.28564453125px 548.571044921875px)',
                            opacity: '0',
                            fill: ["rgba(0,0,0,0)",im+"dr_frame3.png",'0px','0px'],
                            transform: [[],[],[],['0.95649']]
                        },
                        {
                            id: 'dr_motor2Copy',
                            type: 'image',
                            rect: ['418px', '607px', '171px', '188px', 'auto', 'auto'],
                            clip: 'rect(0px 171px 173.714111328125px 0px)',
                            opacity: '0',
                            fill: ["rgba(0,0,0,0)",im+"dr_motor2.png",'0px','0px']
                        },
                        {
                            id: 'dr_wing3Copy',
                            type: 'image',
                            rect: ['120px', '511px', '323px', '261px', 'auto', 'auto'],
                            clip: 'rect(0px 254.428466796875px 152.428466796875px 0px)',
                            opacity: '0',
                            fill: ["rgba(0,0,0,0)",im+"dr_wing3.png",'0px','0px']
                        },
                        {
                            id: 'ROBOTT2',
                            type: 'image',
                            rect: ['258px', '137px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
                        },
                        {
                            id: 'CASE_DEFINITION_GREY1',
                            type: 'image',
                            rect: ['408px', '133px', '892px', '170px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                        },
                        {
                            id: 'Text5',
                            type: 'text',
                            rect: ['447px', '149px', '817px', '164px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Move the cursor &nbsp;over the different parts of the car to get to know the factors that help the dragster to accelerate very quickly.</span></p>",
                            font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['513px', '30px', '390', '57', 'auto', 'auto']
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
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['77px', '-4px', '773px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 773.821533203125px 58.60009765625px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 63px; font-weight: 500; font-family: \'Digital tech\';\">speed factors</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['77px', '-4px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 709.599609375px 61.7998046875px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 63px; font-weight: 500;\">speed factors</span></p>'
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
                            [0,709.599609375,61.7998046875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0140_FacteurVitesse_edgeActions.js");
})("EDGE-6689876");
