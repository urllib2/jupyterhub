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
                            id: 'valider',
                            type: 'group',
                            rect: ['-83px', '786px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['1.00179','1.0122']],
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
                                rect: ['919px', '15px', '111px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Valider</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['221px', '112px', '966', '160', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE2',
                                type: 'image',
                                rect: ['144px', '7px', '907px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['200px', '25px', '812px', '98px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-size: 34px;\">Analyser le porgramme en cliquant&nbsp;<span style=\"font-weight: 700; text-decoration: underline; color: rgb(255, 0, 0);\">Ici</span><span style=\"color: rgb(255, 0, 0);\"> </span>et placer la valeur de sortie afficher à l'écran de la brick à chaque état de capteur.</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1.00)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['0px', '7px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['555px', '33px', '390', '57', 'auto', 'auto'],
                            clip: 'rect(-6.154052734375px 396.15380859375px 67.769287109375px 0px)'
                        },
                        {
                            id: 'star_bot_op',
                            type: 'image',
                            rect: ['1309px', '147px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot1',
                            type: 'image',
                            rect: ['1309px', '147px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'star_bot_op2',
                            type: 'image',
                            rect: ['1397px', '147px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot2',
                            type: 'image',
                            rect: ['1397px', '147px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'Group',
                            type: 'group',
                            rect: ['130px', '434px', '427', '803', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Enfonce',
                                type: 'image',
                                rect: ['16px', '0px', '411px', '231px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"mm_enfonce%C3%A9.png",'0px','0px']
                            },
                            {
                                id: '_1',
                                type: 'text',
                                rect: ['0px', '84px', '285px', '73px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-weight: 700;\">1.</span></p><p style=\"margin: 0px;\">​</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Afficheur',
                                type: 'image',
                                rect: ['38px', '171px', '411px', '668px', 'auto', 'auto'],
                                clip: 'rect(0px 411px 484.3389587402344px 20px)',
                                fill: ["rgba(0,0,0,0)",im+"Afficheur.png",'0px','0px'],
                                transform: [[],[],[],['0.89429','0.89295']]
                            },
                            {
                                id: 'basket_1',
                                type: 'rect',
                                rect: ['125px', '265px', '239px', '132px', 'auto', 'auto'],
                                fill: ["rgba(192,192,192,0)"],
                                stroke: [10,"rgba(255, 0, 0, 0)","solid"]
                            }]
                        },
                        {
                            id: 'Group2',
                            type: 'group',
                            rect: ['1033px', '434px', '410', '369', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Relach',
                                type: 'image',
                                rect: ['0px', '0px', '411px', '231px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"mm_relach%C3%A9.png",'0px','0px']
                            },
                            {
                                id: '_2',
                                type: 'text',
                                rect: ['5px', '71px', '285px', '73px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-weight: 700;\">2.</span></p><p style=\"margin: 0px;\">​</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Afficheur2',
                                type: 'image',
                                rect: ['-30px', '171px', '411px', '668px', 'auto', 'auto'],
                                clip: 'rect(0px 411px 478.3663635253906px 20px)',
                                fill: ["rgba(0,0,0,0)",im+"Afficheur.png",'0px','0px'],
                                transform: [[],[],[],['0.89429','0.89295']]
                            },
                            {
                                id: 'basket_2',
                                type: 'rect',
                                rect: ['64px', '265px', '218px', '132px', 'auto', 'auto'],
                                fill: ["rgba(192,192,192,0)"],
                                stroke: [10,"rgba(255, 0, 0, 0)","solid"]
                            }]
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['81px', '877px', '1449px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                        },
                        {
                            id: 'drop_2',
                            type: 'image',
                            rect: ['537px', '307px', '238px', '152px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Natural.png",'0px','0px']
                        },
                        {
                            id: 'drop_1',
                            type: 'image',
                            rect: ['875px', '307px', '237px', '152px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Tear.png",'0px','0px']
                        },
                        {
                            id: 'RoundRect1',
                            type: 'rect',
                            rect: ['239px', '687px', '265px', '149px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [10,"rgba(255,0,0,0.00)","solid"]
                        },
                        {
                            id: 'RoundRect2',
                            type: 'rect',
                            rect: ['1077px', '687px', '265px', '149px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [10,"rgba(255,0,0,0.00)","solid"]
                        },
                        {
                            id: 'AfficheProg',
                            type: 'rect',
                            rect: ['967px', '137px', '34px', '29px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,0)"],
                            stroke: [10,"rgba(255, 0, 0, 0)","solid"]
                        },
                        {
                            id: 'Text',
                            type: 'text',
                            rect: ['567px', '555px', '458px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-family: MagistralC;\">​</span><span style=\"font-family: MagistralC; font-size: 55px;\">​</span><span style=\"font-family: MagistralC; font-size: 55px; font-weight: 700;\">​</span></p>",
                            font: ['MagistralC', [45, "px"], "rgba(255,0,0,1.00)", "700", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'reset',
                            type: 'group',
                            rect: ['-266px', '786px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGECopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE2.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY2.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy',
                                type: 'text',
                                rect: ['916px', '17px', '111px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Reset</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'q1',
                            type: 'image',
                            rect: ['217px', '254px', '1141px', '741px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"q1.PNG",'0px','0px']
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
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-97px', '-7px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​<span style=\"font-weight: 500;\">Quiz</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['173px', '-7px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\"><span style=\"font-weight: 500;\">Quiz</span></p>',
                            font: ['MS Serif, New York, serif', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 680.7080078125px 57px -78.28271484375px)',
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
                            [0,680.7080078125,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0100_QuizTactile_edgeActions.js");
})("EDGE-6689876");
