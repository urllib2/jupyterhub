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
                            rect: ['303px', '134px', '1046', '219', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['134px', '13px', '787px', '128px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['0px', '0px', '166px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['179px', '32px', '869px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Sélectionner la valeur d'entrée qui imapcte la &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; direction du robot​.</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['364px', '804px', '1040', '82', 'auto', 'auto'],
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
                                rect: ['916px', '17px', '111px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Valider</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Question1',
                            type: 'group',
                            rect: ['221px', '614px', '340', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'E_1',
                                type: 'group',
                                rect: ['403px', '28px', '80', '75', 'auto', 'auto'],
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
                                    rect: ['18px', '16px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            },
                            {
                                id: 'E_0',
                                type: 'group',
                                rect: ['82px', '28px', '80', '75', 'auto', 'auto'],
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
                                    rect: ['18px', '16px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                },
                                {
                                    id: 'Text3Copy',
                                    type: 'text',
                                    rect: ['18px', '80px', '376px', '51px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​ A &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; </p><p style=\"margin: 0px;\">&nbsp; &nbsp; &nbsp;</p>",
                                    align: "left",
                                    font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1.00)", "900", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'E_2',
                                type: 'group',
                                rect: ['720px', '28px', '80', '75', 'auto', 'auto'],
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
                                    rect: ['18px', '16px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                }]
                            }]
                        },
                        {
                            id: 'Question2',
                            type: 'group',
                            rect: ['1265px', '661px', '340', '67px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'E_3',
                                type: 'group',
                                rect: ['0px', '-11px', '80', '67px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'CARRV3',
                                    type: 'image',
                                    rect: ['0px', '-8px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                },
                                {
                                    id: 'CARRR3',
                                    type: 'image',
                                    rect: ['0px', '-8px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                },
                                {
                                    id: 'CARRG3',
                                    type: 'image',
                                    rect: ['0px', '-8px', '80px', '80px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'croix3',
                                    type: 'image',
                                    rect: ['18px', '8px', '46px', '46px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                },
                                {
                                    id: 'Text3Copy4',
                                    type: 'text',
                                    rect: ['18px', '72px', '376px', '45px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​ D</p><p style=\"margin: 0px;\"> &nbsp; &nbsp; &nbsp;</p>",
                                    align: "left",
                                    font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1.00)", "900", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            }]
                        },
                        {
                            id: 'equilibreCopy3',
                            type: 'image',
                            rect: ['1180px', '452px', '289px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a7_stearing.png",'0px','0px']
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['652px', '722px', '184px', '75px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​B</p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text4',
                            type: 'text',
                            rect: ['968px', '722px', '166px', '94px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​C</p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'stars',
                            type: 'group',
                            rect: ['1345px', '119px', '222px', '73', 'auto', 'auto'],
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
                            id: 'RoundRect',
                            type: 'rect',
                            rect: ['282px', '570px', '2px', '2px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'stars_op',
                            type: 'group',
                            rect: ['1338px', '119px', '232', '73', 'auto', 'auto'],
                            c: [
                            {
                                id: 'star_bot_op',
                                type: 'image',
                                rect: ['0px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Attention',
                            type: 'text',
                            rect: ['599px', '807px', '441px', '29px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 40px;\">Cocher une reponse !</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [37, "px"], "rgba(96,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'resultat',
                            type: 'group',
                            rect: ['339px', '316px', '1025', '111', 'auto', 'auto'],
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
                            id: 'equilibre',
                            type: 'image',
                            rect: ['847px', '447px', '289px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a7_stearing.png",'0px','0px']
                        },
                        {
                            id: 'equilibreCopy',
                            type: 'image',
                            rect: ['521px', '448px', '289px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a7_stearing.png",'0px','0px']
                        },
                        {
                            id: 'RoundRect2Copy',
                            type: 'rect',
                            rect: ['640px', '487px', '38px', '103px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            opacity: '1',
                            fill: ["rgba(255,255,255,0.00)"],
                            stroke: [5,"rgba(255,0,0,1.00)","solid"]
                        },
                        {
                            id: 'RoundRect2Copy2',
                            type: 'rect',
                            rect: ['1009px', '487px', '38px', '103px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            opacity: '1',
                            fill: ["rgba(255,255,255,0.00)"],
                            stroke: [5,"rgba(255,0,0,1.00)","solid"]
                        },
                        {
                            id: 'RoundRect2Copy3',
                            type: 'rect',
                            rect: ['1383px', '492px', '38px', '103px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            opacity: '1',
                            fill: ["rgba(255,255,255,0.00)"],
                            stroke: [5,"rgba(255,0,0,1.00)","solid"]
                        },
                        {
                            id: 'equilibreCopy2',
                            type: 'image',
                            rect: ['199px', '447px', '289px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a7_stearing.png",'0px','0px']
                        },
                        {
                            id: 'RoundRect2',
                            type: 'rect',
                            rect: ['276px', '487px', '38px', '103px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            opacity: '1',
                            fill: ["rgba(255,255,255,0.00)"],
                            stroke: [5,"rgba(255,0,0,1.00)","solid"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_2',
                            type: 'rect',
                            rect: ['744px', '25px', '390', '57', 'auto', 'auto']
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

                    ]
                }
            },
            "TITRE_2": {
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
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">quiz</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['-4px', '0px', '766px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 650px 57px -155.55541992188px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-weight: 500;\">Quiz</span></p>'
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
                            '0px',
                            '0px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0205_QuizProg_edgeActions.js");
})("EDGE-6689876");
