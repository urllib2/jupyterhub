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
                            id: 'Into',
                            type: 'group',
                            rect: ['211px', '111px', '958px', '184px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['99px', '0px', '958px', '216px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'RobotS',
                                type: 'image',
                                rect: ['-61px', '20px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['129px', '24px', '910px', '60px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Saisir la valeur qui manque dans les blocs suivant afin que le robot réalise la mission suivante:</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​-Clignote en </span><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">\"orangé\"</span><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> pendant </span><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">3 secondes</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​-Avancer <b>2 rotations</b></span><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">&nbsp;avec une </span><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">puissance 50</span></p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['387px', '825px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 984px 82px 834.17724609375px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['847px', '7px', '131px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['847px', '7px', '131px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
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
                            id: 'Quiz2',
                            type: 'image',
                            rect: ['147px', '490px', '1115px', '269px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Quiz2.PNG",'0px','0px']
                        },
                        {
                            id: 'star_bot_op',
                            type: 'image',
                            rect: ['1281px', '154px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot_op2',
                            type: 'image',
                            rect: ['1353px', '154px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot_op3',
                            type: 'image',
                            rect: ['1424px', '154px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot1',
                            type: 'image',
                            rect: ['1281px', '154px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'star_bot2',
                            type: 'image',
                            rect: ['1353px', '154px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'star_bot3',
                            type: 'image',
                            rect: ['1424px', '154px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'Couleur',
                            type: 'group',
                            rect: ['276px', '395px', '246', '123', 'auto', 'auto'],
                            c: [
                            {
                                id: 'RoundRect2',
                                type: 'rect',
                                rect: ['35px', '6px', '216px', '93px', 'auto', 'auto'],
                                borderRadius: ["10px", "10px", "10px", "10px"],
                                fill: ["rgba(192,192,192,0.00)"],
                                stroke: [10,"rgba(54,10,241,1.00)","solid"]
                            },
                            {
                                id: 'Text1',
                                type: 'text',
                                rect: ['49px', '35px', '207px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'h2',
                                type: 'image',
                                rect: ['-182px', '65px', '585px', '512px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"h2.PNG",'0px','0px']
                            },
                            {
                                id: 'Rectangle',
                                type: 'rect',
                                rect: ['44px', '140px', '226px', '140px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(192,192,192,0)"],
                                stroke: [10,"rgba(255,255,255,0.00)","solid"]
                            },
                            {
                                id: 'fleche4',
                                type: 'image',
                                rect: ['98px', '62px', '169px', '172px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"33.png",'0px','0px'],
                                transform: [[],['90'],[],['0.64866','0.64866']]
                            }]
                        },
                        {
                            id: 'pointer',
                            type: 'image',
                            rect: ['391px', '638px', '38px', '57px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
                        },
                        {
                            id: 'Secondes',
                            type: 'group',
                            rect: ['690px', '437px', '246', '123', 'auto', 'auto'],
                            c: [
                            {
                                id: 'RoundRect1',
                                type: 'rect',
                                rect: ['-114px', '-36px', '216px', '93px', 'auto', 'auto'],
                                borderRadius: ["10px", "10px", "10px", "10px"],
                                fill: ["rgba(192,192,192,0.00)"],
                                stroke: [10,"rgba(54,10,241,1.00)","solid"]
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['-100px', '-7px', '205px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'fleche3',
                                type: 'image',
                                rect: ['-35px', '12px', '169px', '186px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"33.png",'0px','0px'],
                                transform: [[],['90'],[],['0.64864','0.64864']]
                            }]
                        },
                        {
                            id: 'Puissance',
                            type: 'group',
                            rect: ['822px', '398px', '246', '123', 'auto', 'auto'],
                            c: [
                            {
                                id: 'RoundRect3',
                                type: 'rect',
                                rect: ['0px', '3px', '216px', '93px', 'auto', 'auto'],
                                borderRadius: ["10px", "10px", "10px", "10px"],
                                fill: ["rgba(192,192,192,0.00)"],
                                stroke: [10,"rgba(54,10,241,1.00)","solid"]
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['14px', '32px', '205px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'fleche2',
                                type: 'image',
                                rect: ['122px', '57px', '160px', '169px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"33.png",'0px','0px'],
                                transform: [[],['90'],[],['0.64866','0.64866']]
                            }]
                        },
                        {
                            id: 'Rotation',
                            type: 'group',
                            rect: ['1063px', '396px', '246', '123', 'auto', 'auto'],
                            c: [
                            {
                                id: 'RoundRect4',
                                type: 'rect',
                                rect: ['0px', '5px', '216px', '93px', 'auto', 'auto'],
                                borderRadius: ["10px", "10px", "10px", "10px"],
                                fill: ["rgba(192,192,192,0.00)"],
                                stroke: [10,"rgba(54,10,241,1.00)","solid"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['14px', '34px', '210px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'fleche',
                                type: 'image',
                                rect: ['-52px', '46px', '156px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"33.png",'0px','0px'],
                                transform: [[],['90'],[],['0.64866','0.64866']]
                            }]
                        },
                        {
                            id: 'Message',
                            type: 'text',
                            rect: ['547px', '740px', '829px', '92px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 50px; color: rgb(0, 0, 0); font-family: MagistralC;\">Vous avez &nbsp; &nbsp; &nbsp;/ &nbsp; &nbsp; &nbsp; &nbsp;bonne reponse &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;</span>​</p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [48, "px"], "rgba(163,10,10,1)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Message2',
                            type: 'text',
                            rect: ['590px', '825px', '488px', '75px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 50px; color: rgb(159, 7, 149); font-family: MagistralC;\">Verifier votre valeurs </span><span style=\"font-size: 50px; color: rgb(0, 0, 0); font-family: MagistralC;\">&nbsp; &nbsp; &nbsp; &nbsp;&nbsp;</span>​</p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [48, "px"], "rgba(163,10,10,1)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Note',
                            type: 'text',
                            rect: ['792px', '705px', '64px', '92px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            align: "left",
                            font: ['MagistralC', [91, "px"], "rgba(193,128,2,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Total',
                            type: 'text',
                            rect: ['891px', '705px', '54px', '92px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-family: MagistralC; color: rgb(193, 128, 2); font-size: 91px;\">4</span><span style=\"font-family: MagistralC; color: rgb(171, 127, 17);\">​</span></p>",
                            align: "left",
                            font: ['MagistralC', [60, "px"], "rgba(227,181,66,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['516px', '31px', '390', '57', 'auto', 'auto']
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
                    duration: 11192,
                    autoPlay: false,
                    data: [
                        [
                            "eid222",
                            "top",
                            11192,
                            0,
                            "linear",
                            "${pointer}",
                            '638px',
                            '638px'
                        ],
                        [
                            "eid225",
                            "width",
                            11192,
                            0,
                            "linear",
                            "${Text3}",
                            '205px',
                            '205px'
                        ],
                        [
                            "eid226",
                            "width",
                            11192,
                            0,
                            "linear",
                            "${Text4}",
                            '210px',
                            '210px'
                        ],
                        [
                            "eid224",
                            "width",
                            11192,
                            0,
                            "linear",
                            "${Text2}",
                            '205px',
                            '205px'
                        ],
                        [
                            "eid214",
                            "width",
                            11192,
                            0,
                            "linear",
                            "${pointer}",
                            '38px',
                            '38px'
                        ],
                        [
                            "eid221",
                            "left",
                            11192,
                            0,
                            "linear",
                            "${pointer}",
                            '391px',
                            '391px'
                        ],
                        [
                            "eid223",
                            "width",
                            11192,
                            0,
                            "linear",
                            "${Text1}",
                            '207px',
                            '207px'
                        ],
                        [
                            "eid213",
                            "height",
                            11192,
                            0,
                            "linear",
                            "${pointer}",
                            '57px',
                            '57px'
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
                            rect: ['-62px', '0px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Quiz</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['208px', '-1px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Quiz</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
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
                            [0,519.83837890625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0110_QuizSaisie_edgeActions.js");
})("EDGE-6689876");
