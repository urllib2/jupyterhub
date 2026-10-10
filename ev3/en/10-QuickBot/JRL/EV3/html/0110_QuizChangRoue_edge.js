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
                scaleToFit: "none",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['86px', '-4px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'resultat',
                            type: 'group',
                            rect: ['339px', '360px', '1025', '111', 'auto', 'auto'],
                            clip: 'rect(0px 976.2822265625px 110px 0px)',
                            c: [
                            {
                                id: 'message',
                                type: 'text',
                                rect: ['55px', '28px', '1025px', '83px', 'auto', 'auto'],
                                clip: 'rect(0px 872.918212890625px 83px 0px)',
                                text: "<p style=\"margin: 0px;\">You have &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;good answers</p>",
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
                            id: 'RectangleCopy3',
                            type: 'rect',
                            rect: ['280px', '585px', '925px', '99px', 'auto', 'auto'],
                            fill: ["rgba(93,216,223,0.28)"],
                            stroke: [2,"rgba(35,167,169,1.00)","solid"],
                            c: [
                            {
                                id: 'Q2',
                                type: 'text',
                                rect: ['16px', '16px', '758px', '130px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">The distance traveled by the robot​</p>",
                                align: "left",
                                font: ['\'Palatino Linotype\', \'Book Antigua\', Palatino, serif', [34, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'RectangleCopy2',
                            type: 'rect',
                            rect: ['281px', '471px', '925px', '99px', 'auto', 'auto'],
                            fill: ["rgba(93,216,223,0.28)"],
                            stroke: [2,"rgba(35,167,169,1.00)","solid"],
                            c: [
                            {
                                id: 'Q1',
                                type: 'text',
                                rect: ['15px', '17px', '789px', '130px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 35px;\">Robot speed</span></p>",
                                align: "left",
                                font: ['\'Palatino Linotype\', \'Book Antigua\', Palatino, serif', [34, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'RectangleCopy',
                            type: 'rect',
                            rect: ['281px', '696px', '925px', '99px', 'auto', 'auto'],
                            fill: ["rgba(93,216,223,0.28)"],
                            stroke: [2,"rgba(35,167,169,1.00)","solid"]
                        },
                        {
                            id: 'rep2_no',
                            type: 'image',
                            rect: ['1076px', '593px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                        },
                        {
                            id: 'rep1_yes',
                            type: 'image',
                            rect: ['1076px', '481px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                        },
                        {
                            id: 'rep2_yes',
                            type: 'image',
                            rect: ['1076px', '593px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                        },
                        {
                            id: 'rep_2',
                            type: 'image',
                            rect: ['1075px', '593px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                        },
                        {
                            id: 'rep3_yes',
                            type: 'image',
                            rect: ['1076px', '706px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                        },
                        {
                            id: 'rep3_no',
                            type: 'image',
                            rect: ['1076px', '706px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                        },
                        {
                            id: 'rep_3',
                            type: 'image',
                            rect: ['1076px', '706px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                        },
                        {
                            id: 'rep1_no',
                            type: 'image',
                            rect: ['1076px', '481px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                        },
                        {
                            id: 'rep_1',
                            type: 'image',
                            rect: ['1076px', '481px', '94px', '85px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                        },
                        {
                            id: 'Yes',
                            type: 'image',
                            rect: ['1281px', '491px', '50px', '50px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"Yes2.PNG",'0px','0px']
                        },
                        {
                            id: 'No_1',
                            type: 'image',
                            rect: ['1215px', '491px', '50px', '50px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"No2.PNG",'0px','0px']
                        },
                        {
                            id: 'Yes_2',
                            type: 'image',
                            rect: ['1280px', '604px', '50px', '50px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"Yes2.PNG",'0px','0px']
                        },
                        {
                            id: 'No_22',
                            type: 'image',
                            rect: ['1214px', '604px', '50px', '50px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"No2.PNG",'0px','0px']
                        },
                        {
                            id: 'Yes_33',
                            type: 'image',
                            rect: ['1281px', '724px', '50px', '50px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"Yes2.PNG",'0px','0px']
                        },
                        {
                            id: 'No_3',
                            type: 'image',
                            rect: ['1215px', '724px', '50px', '50px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"No2.PNG",'0px','0px']
                        },
                        {
                            id: 'Q3',
                            type: 'text',
                            rect: ['296px', '716px', '881px', '112px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 35px;\">Motor rotation number</span></p>",
                            align: "left",
                            font: ['\'Palatino Linotype\', \'Book Antigua\', Palatino, serif', [34, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'intro',
                            type: 'group',
                            rect: ['206', '125', '978px', '257', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['128px', '13px', '848px', '185px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['192px', '34px', '383px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Changing the robot's wheels to these wheels will impact:​</span></p>",
                                align: "left",
                                font: ['Palatino Linotype, Book Antigua, Palatino, serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'a7_genbo',
                                type: 'image',
                                rect: ['567px', '18px', '371px', '257px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"a7_genbo.png",'0px','0px']
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['2px', '36px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'reset',
                            type: 'group',
                            rect: ['190px', '832px', '1040', '82', 'auto', 'auto'],
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
                            id: 'Text',
                            type: 'text',
                            rect: ['704px', '812px', '202px', '43px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            align: "left",
                            font: ['\'Palatino Linotype\', \'Book Antigua\', Palatino, serif', [38, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['335px', '832px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGECopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy2',
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
                            id: 'stars_op',
                            type: 'group',
                            rect: ['1226px', '120px', '232', '73', 'auto', 'auto'],
                            c: [
                            {
                                id: 'star_bot_op',
                                type: 'image',
                                rect: ['0px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                            },
                            {
                                id: 'star_bot_op2',
                                type: 'image',
                                rect: ['86px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                            },
                            {
                                id: 'star_bot_op3',
                                type: 'image',
                                rect: ['168px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'stars',
                            type: 'group',
                            rect: ['1233px', '120px', '222px', '73', 'auto', 'auto'],
                            c: [
                            {
                                id: 'star_bot1',
                                type: 'image',
                                rect: ['-7px', '0px', '64px', '73px', 'auto', 'auto'],
                                opacity: '1',
                                fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                            },
                            {
                                id: 'star_bot2',
                                type: 'image',
                                rect: ['79px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                            },
                            {
                                id: 'star_bot3',
                                type: 'image',
                                rect: ['161px', '0px', '64px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['535px', '23px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Essai',
                            type: 'text',
                            rect: ['280px', '820px', '688px', '75px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            font: ['MagistralC', [55, "px"], "rgba(255,0,0,1.00)", "500", "none", "", "break-word", "normal"]
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
                            "eid380",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Text5}",
                            '192px',
                            '192px'
                        ],
                        [
                            "eid326",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Q2}",
                            '16px',
                            '16px'
                        ],
                        [
                            "eid375",
                            "top",
                            0,
                            0,
                            "linear",
                            "${RectangleCopy3}",
                            '585px',
                            '585px'
                        ],
                        [
                            "eid288",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Q1}",
                            '15px',
                            '15px'
                        ],
                        [
                            "eid381",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Text5}",
                            '34px',
                            '34px'
                        ],
                        [
                            "eid323",
                            "height",
                            0,
                            0,
                            "linear",
                            "${Text5}",
                            '57px',
                            '57px'
                        ],
                        [
                            "eid343",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Q2}",
                            '16px',
                            '16px'
                        ],
                        [
                            "eid360",
                            "width",
                            0,
                            0,
                            "linear",
                            "${Q1}",
                            '789px',
                            '789px'
                        ],
                        [
                            "eid328",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Q1}",
                            '17px',
                            '17px'
                        ],
                        [
                            "eid378",
                            "width",
                            0,
                            0,
                            "linear",
                            "${Text5}",
                            '383px',
                            '383px'
                        ],
                        [
                            "eid310",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Q3}",
                            '296px',
                            '296px'
                        ],
                        [
                            "eid361",
                            "background-color",
                            0,
                            0,
                            "linear",
                            "${RectangleCopy3}",
                            'rgba(93,216,223,0.28)',
                            'rgba(93,216,223,0.28)'
                        ],
                        [
                            "eid376",
                            "top",
                            0,
                            0,
                            "linear",
                            "${RectangleCopy2}",
                            '471px',
                            '471px'
                        ],
                        [
                            "eid282",
                            "font-size",
                            0,
                            0,
                            "linear",
                            "${Text5}",
                            '32px',
                            '32px'
                        ],
                        [
                            "eid362",
                            "background-color",
                            0,
                            0,
                            "linear",
                            "${RectangleCopy2}",
                            'rgba(93,216,223,0.28)',
                            'rgba(93,216,223,0.28)'
                        ],
                        [
                            "eid313",
                            "left",
                            0,
                            0,
                            "linear",
                            "${RectangleCopy3}",
                            '280px',
                            '280px'
                        ],
                        [
                            "eid377",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Q3}",
                            '716px',
                            '716px'
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0110_QuizChangRoue_edgeActions.js");
})("EDGE-6689876");
