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
                            rect: ['287px', '131px', '887', '160', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['99px', '-18px', '887px', '167px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'RobotS',
                                type: 'image',
                                rect: ['-63px', '-11px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['163px', '14px', '683px', '66px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Après l'exécution de ce programme, quelle est la distance parcourue par ce robot ?</span></p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            }]
                        },
                        {
                            id: 'prog',
                            type: 'group',
                            rect: ['319px', '377px', '478px', '175px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'blo5',
                                type: 'image',
                                rect: ['0px', '5px', '143px', '161px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"blo5.png",'0px','0px']
                            },
                            {
                                id: 'a7_stearing',
                                type: 'image',
                                rect: ['121px', '0px', '352px', '167px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"a7_stearing.png",'0px','0px']
                            },
                            {
                                id: 'Rectangle3',
                                type: 'rect',
                                rect: ['344px', '114px', '17px', '13px', 'auto', 'auto'],
                                fill: ["rgba(227,227,227,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'Rectangle',
                                type: 'rect',
                                rect: ['326px', '127px', '47px', '14px', 'auto', 'auto'],
                                fill: ["rgba(192,192,192,1)"],
                                stroke: [0,"rgba(0,0,0,1)","none"]
                            },
                            {
                                id: 'Rectangle2',
                                type: 'rect',
                                rect: ['335px', '125px', '33px', '6px', 'auto', 'auto'],
                                fill: ["rgba(192,192,192,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['340px', '112px', '33px', '26px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 20px;\">2</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'a7_genbo',
                            type: 'image',
                            rect: ['818px', '305px', '473px', '327px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a7_genbo.png",'0px','0px']
                        },
                        {
                            id: 'message',
                            type: 'text',
                            rect: ['426px', '623px', '1025px', '83px', 'auto', 'auto'],
                            clip: 'rect(0px 872.918212890625px 83px 0px)',
                            text: "<p style=\"margin: 0px;\">​<span style=\"color: rgb(205, 135, 0); font-size: 53px;\">message</span></p>",
                            font: ['MagistralC', [51, "px"], "rgba(5,160,0,1.00)", "600", "none", "", "break-word", "normal"],
                            transform: [[],[],[],['1.10575']]
                        },
                        {
                            id: 'message2',
                            type: 'text',
                            rect: ['525px', '623px', '1025px', '83px', 'auto', 'auto'],
                            clip: 'rect(0px 872.918212890625px 83px 0px)',
                            text: "<p style=\"margin: 0px;\">​<span style=\"color: rgb(205, 135, 0); font-size: 53px;\">message</span></p>",
                            font: ['MagistralC', [51, "px"], "rgba(5,160,0,1.00)", "600", "none", "", "break-word", "normal"],
                            transform: [[],[],[],['1.10575']]
                        },
                        {
                            id: 'ligne_1',
                            type: 'group',
                            rect: ['478px', '760px', '559px', '82', 'auto', 'auto'],
                            c: [
                            {
                                id: 'pack22',
                                type: 'image',
                                rect: ['0px', '0px', '773px', '82px', 'auto', 'auto'],
                                clip: 'rect(0px 525px 82px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"pack22.png",'0px','0px']
                            },
                            {
                                id: 'Text1',
                                type: 'text',
                                rect: ['289px', '10px', '218px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'test',
                                type: 'text',
                                rect: ['35px', '8px', '210px', '72px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; line-height: 30px;\">​<span style=\"font-size: 28px; font-family: 'Arial Black', Gadget, sans-serif; color: rgb(38, 41, 120);\">Distance en centimètre</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['247px', '763px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 834.17724609375px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['847px', '7px', '131px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE4.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['847px', '7px', '131px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY4.png",'0px','0px']
                            },
                            {
                                id: 'Text311',
                                type: 'text',
                                rect: ['856px', '16px', '180px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Valider</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'message1',
                            type: 'text',
                            rect: ['965px', '836px', '425px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            align: "left",
                            font: ['Palatino Linotype, Book Antigua, Palatino, serif', [39, "px"], "rgba(163,10,10,1.00)", "600", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text3',
                            type: 'text',
                            rect: ['477px', '688px', '507px', '35px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"color: rgb(0, 0, 0); font-size: 29px; font-family: Arial, Helvetica, sans-serif; letter-spacing: 2px; font-weight: 700;\">Saisir votre réponse :</span></p>",
                            align: "left",
                            font: ['MagistralC', [51, "px"], "rgba(63,93,255,1)", "600", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['516px', '31px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'star_bot_op',
                            type: 'image',
                            rect: ['1338px', '138px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot1',
                            type: 'image',
                            rect: ['1338px', '138px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'Hint',
                            type: 'group',
                            rect: ['1010px', '236px', '640px', '262px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Hint_green',
                                type: 'image',
                                rect: ['-85px', '-3px', '656px', '0px', 'auto', 'auto'],
                                clip: 'rect(0px 656px 241.58349609375px 0px)',
                                opacity: '0.3',
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px'],
                                transform: [[],[],[],['0.78659','0.63158']]
                            },
                            {
                                id: 'TextCopy2',
                                type: 'text',
                                rect: ['27px', '-11px', '433px', '114px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><br></p><p style=\"margin: 0px;\"><span style=\"font-weight: 700;\">la circonferance de cette roue est 21.6 cm ​</span></p>",
                                align: "left",
                                font: ['\'Arial Black\', Gadget, sans-serif', [24, "px"], "rgba(0,0,0,0.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'indices',
                                type: 'image',
                                rect: ['58px', '-32px', '150px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"indices.png",'0px','0px']
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
                        [
                            "eid218",
                            "color",
                            1648,
                            352,
                            "linear",
                            "${TextCopy2}",
                            'rgba(0,0,0,0)',
                            'rgba(0,0,0,1.00)'
                        ],
                        [
                            "eid222",
                            "height",
                            1000,
                            905,
                            "linear",
                            "${Hint_green}",
                            '0px',
                            '228px'
                        ],
                        [
                            "eid219",
                            "top",
                            1000,
                            1000,
                            "linear",
                            "${Hint_green}",
                            '-3px',
                            '-53px'
                        ],
                        [
                            "eid220",
                            "opacity",
                            1000,
                            1000,
                            "linear",
                            "${Hint_green}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid221",
                            "left",
                            1000,
                            0,
                            "linear",
                            "${Hint_green}",
                            '-85px',
                            '-85px'
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
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['-62px', '0px', '655px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Quiz</p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['208px', '-1px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Quiz</span></p>'
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0120_QuizCalculDistance_edgeActions.js");
})("EDGE-6689876");
