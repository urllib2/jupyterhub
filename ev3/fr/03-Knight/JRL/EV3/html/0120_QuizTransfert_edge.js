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
                            id: 'CASE_DEFINITION_GREEN',
                            type: 'image',
                            rect: ['449px', '142px', '812px', '162px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                        },
                        {
                            id: 'Text5',
                            type: 'text',
                            rect: ['497px', '176px', '869px', '165px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Cocher les méthodes utlisables pour télécharger </span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">un programme dans la brique intelligente</span></p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Question1',
                            type: 'group',
                            rect: ['175px', '629px', '340', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'E_1',
                                type: 'group',
                                rect: ['458px', '28px', '80', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV1',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR1',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG1',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix1',
                                    type: 'image',
                                    rect: ['17px', '15px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            },
                            {
                                id: 'E_0',
                                type: 'group',
                                rect: ['130px', '28px', '80', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV0',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR0',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG0',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix0',
                                    type: 'image',
                                    rect: ['17px', '15px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                },
                                {
                                    id: 'Text3Copy',
                                    type: 'text',
                                    rect: ['-53px', '79px', '376px', '51px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 500; font-size: 34px;\">&nbsp;Cable USB &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 500;\">&nbsp; &nbsp; &nbsp;</span></p>",
                                    align: "left",
                                    font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1.00)", "900", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'E_2',
                                type: 'group',
                                rect: ['760px', '28px', '80', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV2',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR2',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG2',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix2',
                                    type: 'image',
                                    rect: ['17px', '15px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            }]
                        },
                        {
                            id: 'Question2',
                            type: 'group',
                            rect: ['1216px', '683px', '340', '67px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'E_3',
                                type: 'group',
                                rect: ['0px', '-26px', '80', '67px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV3',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR3',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG3',
                                    type: 'image',
                                    rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix3',
                                    type: 'image',
                                    rect: ['17px', '12px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                },
                                {
                                    id: 'Text3Copy4',
                                    type: 'text',
                                    rect: ['-30px', '81px', '376px', '45px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 24px;\"> </span><span style=\"font-weight: 500; font-size: 34px;\">Bluetooth</span></p><p style=\"margin: 0px;\"> &nbsp; &nbsp; &nbsp;</p>",
                                    align: "left",
                                    font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1.00)", "900", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['635px', '739px', '184px', '75px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 500; font-size: 34px;\">WiFi</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text4',
                            type: 'text',
                            rect: ['901px', '739px', '222px', '94px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 500; font-size: 34px;\">Infrarouge</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'ROBOT06',
                            type: 'image',
                            rect: ['311px', '145px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                        },
                        {
                            id: 'stars',
                            type: 'group',
                            rect: ['1384px', '119px', '222px', '73', 'auto', 'auto'],
                            c: [
                            {
                                id: 'star_bot1',
                                type: 'image',
                                rect: ['-7px', '0px', '64px', '73px', 'auto', 'auto'],
                                opacity: '1',
                                fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'stars_op',
                            type: 'group',
                            rect: ['1377px', '119px', '232', '73', 'auto', 'auto'],
                            c: [
                            {
                                id: 'star_bot_op',
                                type: 'image',
                                rect: ['0px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['307px', '817px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['1.09286','1.1418']],
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
                                rect: ['916px', '17px', '111px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Valider</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Images',
                            type: 'group',
                            rect: ['278px', '410px', '1056px', '234px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'usbCable',
                                type: 'image',
                                rect: ['0px', '74px', '130px', '130px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_usbCable.png",'0px','0px']
                            },
                            {
                                id: 'wireless',
                                type: 'image',
                                rect: ['312px', '74px', '130px', '130px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_wireless.png",'0px','0px']
                            },
                            {
                                id: 'infrared',
                                type: 'image',
                                rect: ['654px', '74px', '111px', '111px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_infrared.png",'0px','0px']
                            },
                            {
                                id: 'bluetooth2',
                                type: 'image',
                                rect: ['926px', '74px', '80px', '138px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_bluetooth2.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'resultat',
                            type: 'group',
                            rect: ['383px', '334px', '1025', '111', 'auto', 'auto'],
                            clip: 'rect(0px 976.2822265625px 110px 0px)',
                            c: [
                            {
                                id: 'message',
                                type: 'text',
                                rect: ['55px', '28px', '1025px', '83px', 'auto', 'auto'],
                                clip: 'rect(0px 872.918212890625px 83px 0px)',
                                text: "<p style=\"margin: 0px;\">Vous avez &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; bonnes reponses</p>",
                                font: ['MagistralC', [49, "px"], "rgba(0,0,0,1)", "600", "none", "", "break-word", "normal"],
                                transform: [[],[],[],['1.10575']]
                            },
                            {
                                id: 'nombre',
                                type: 'text',
                                rect: ['267px', '0px', '207px', '75px', 'auto', 'auto'],
                                clip: 'rect(0px 222.76953125px 95.512939453125px 0px)',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 800; font-size: 87px; color: rgb(193, 128, 2);\">0</span></p>",
                                align: "left",
                                font: ['MagistralC', [91, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Attention',
                            type: 'text',
                            rect: ['584px', '794px', '441px', '29px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 40px;\">Cocher une reponse !</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [37, "px"], "rgba(96,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0120_QuizTransfert_edgeActions.js");
})("EDGE-6689876");
