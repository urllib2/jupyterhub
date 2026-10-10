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
            js+"jquery-ui.min.js",
            js+"jquery.zoom-min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "both",
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
                            id: 'Intro',
                            type: 'group',
                            rect: ['194px', '133px', '1045', '168', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['144px', '7px', '842px', '153px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                            },
                            {
                                id: 'Definition',
                                type: 'text',
                                rect: ['180px', '38px', '865px', '112px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Identifier chaque pièce en la plaçant dans la zone adéquate​</p>",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'ROBOT07',
                                type: 'image',
                                rect: ['0px', '0px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'baskets',
                            type: 'group',
                            rect: ['308px', '587px', '1018', '207', 'auto', 'auto'],
                            c: [
                            {
                                id: 'basket_3',
                                type: 'rect',
                                rect: ['536px', '13px', '141px', '150px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"],
                                transform: [[],[],[],['0.71']]
                            },
                            {
                                id: 'basket_4',
                                type: 'rect',
                                rect: ['822px', '13px', '100px', '150px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'basket_2',
                                type: 'rect',
                                rect: ['268px', '13px', '145px', '150px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"],
                                transform: [[],[],[],['0.69']]
                            },
                            {
                                id: 'basket_1',
                                type: 'rect',
                                rect: ['3px', '13px', '147px', '150px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [0,"rgba(0,0,0,1)","none"],
                                transform: [[],[],[],['0.68']]
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['445px', '675px', '1040', '82', 'auto', 'auto'],
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
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Valider</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'drop_6',
                            type: 'image',
                            rect: ['126px', '359px', '188px', '195px', 'auto', 'auto'],
                            clip: 'rect(0px 188px 140.58837890625px 0px)',
                            fill: ["rgba(0,0,0,0)",im+"Ultrason.png",'0px','0px']
                        },
                        {
                            id: 'drop_3',
                            type: 'image',
                            rect: ['1221px', '390px', '238px', '134px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"motor.png",'0px','0px']
                        },
                        {
                            id: 'drop_2',
                            type: 'image',
                            rect: ['798px', '357px', '203px', '174px', 'auto', 'auto'],
                            clip: 'rect(20.58837890625px 203px 151.96017456054688px -4.41162109375px)',
                            fill: ["rgba(0,0,0,0)",im+"poutre.png",'0px','0px'],
                            transform: [[],[],[],['0.82759','0.92529']]
                        },
                        {
                            id: 'drop_4',
                            type: 'image',
                            rect: ['329px', '384px', '222px', '134px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Chevis.png",'0px','0px']
                        },
                        {
                            id: 'drop_1',
                            type: 'image',
                            rect: ['556px', '384px', '222px', '134px', 'auto', 'auto'],
                            clip: 'rect(0px 205.8232421875px 134px 11.7646484375px)',
                            fill: ["rgba(0,0,0,0)",im+"axepair.png",'0px','0px'],
                            transform: [[],[],[],['0.98484','1.1194']]
                        },
                        {
                            id: 'drop_5',
                            type: 'image',
                            rect: ['991px', '344px', '276px', '183px', 'auto', 'auto'],
                            clip: 'rect(-0.5734687447547913px 275.7968444824219px 184.0271453857422px -0.9347569346427917px)',
                            fill: ["rgba(0,0,0,0)",im+"roueGeante.png",'0px','0px'],
                            transform: [[],[],[],['0.57177','0.73551']]
                        },
                        {
                            id: 'reset',
                            type: 'group',
                            rect: ['445px', '766px', '1040', '82', 'auto', 'auto'],
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
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Reset</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'stars_op',
                            type: 'group',
                            rect: ['1245px', '150px', '232px', '73px', 'auto', 'auto'],
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
                            rect: ['1252px', '150px', '222px', '73px', 'auto', 'auto'],
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
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'zones',
                            type: 'group',
                            rect: ['237px', '554px', '1077', '242', 'auto', 'auto'],
                            c: [
                            {
                                id: 'sepCopy3',
                                type: 'image',
                                rect: ['773px', '2px', '57px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'sepCopy2',
                                type: 'image',
                                rect: ['247px', '3px', '57px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'sepCopy',
                                type: 'image',
                                rect: ['510px', '1px', '57px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'sep',
                                type: 'image',
                                rect: ['773px', '2px', '57px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'pack',
                                type: 'image',
                                rect: ['1px', '2px', '1077px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pack.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'vertrouge',
                            type: 'group',
                            rect: ['258px', '567px', '1036', '215', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CARR_VERT0',
                                type: 'image',
                                rect: ['0px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Vrai2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_ROUGE0',
                                type: 'image',
                                rect: ['0px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Faux2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_VERT1',
                                type: 'image',
                                rect: ['261px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Vrai2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_ROUGE1',
                                type: 'image',
                                rect: ['262px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Faux2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_VERT2',
                                type: 'image',
                                rect: ['525px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Vrai2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_ROUGE2',
                                type: 'image',
                                rect: ['525px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Faux2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_VERT3',
                                type: 'image',
                                rect: ['789px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Vrai2.png",'0px','0px']
                            },
                            {
                                id: 'CARR_ROUGE3',
                                type: 'image',
                                rect: ['789px', '0px', '247px', '215px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Cadre_Faux2.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Group',
                            type: 'group',
                            rect: ['286', '803', '1025', '99', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['0px', '8px', '189px', '91px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 31px; color: rgb(0, 0, 0); font-weight: 500;\">Axe, 10 modules​</span></p>",
                                align: "left",
                                font: ['Palatino Linotype, Book Antigua, Palatino, serif', [40, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "32px", "", "none"]
                            },
                            {
                                id: 'TextCopy',
                                type: 'text',
                                rect: ['255px', '0px', '168px', '90px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 31px; color: rgb(0, 0, 0); font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Poutre, 15 modules​</span></p>",
                                align: "left",
                                font: ['Palatino Linotype, Book Antigua, Palatino, serif', [40, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "32px", "", "none"]
                            },
                            {
                                id: 'TextCopy2',
                                type: 'text',
                                rect: ['518px', '23px', '204px', '61px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 31px; font-weight: 500; color: rgb(0, 0, 0); font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Gros moteur&nbsp;</span></p>",
                                align: "left",
                                font: ['Palatino Linotype, Book Antigua, Palatino, serif', [40, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "32px", "", "none"]
                            },
                            {
                                id: 'TextCopy3',
                                type: 'text',
                                rect: ['761px', '8px', '264px', '82px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-size: 31px; color: rgb(0, 0, 0);\">Cheville à friction, 3 modules​</span></p>",
                                align: "left",
                                font: ['Palatino Linotype, Book Antigua, Palatino, serif', [40, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "32px", "", "none"]
                            }]
                        },
                        {
                            id: 'Txt',
                            type: 'group',
                            rect: ['124px', '360px', '1360', '187px', 'auto', 'auto'],
                            transform: [[],[],[],['0.98235']],
                            c: [
                            {
                                id: 'Rectangle',
                                type: 'rect',
                                rect: ['-11px', '0px', '1372px', '195px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,1.00)"],
                                stroke: [0,"rgba(0,0,0,1)","none"],
                                transform: [[],[],[],['1.02058']]
                            },
                            {
                                id: 'resultat',
                                type: 'group',
                                rect: ['201px', '-8px', '1025', '124', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'message',
                                    type: 'text',
                                    rect: ['0px', '41px', '1025px', '83px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">vous avez &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;<font color=\"#c18002\"><span style=\"font-size: 91px;\">&nbsp; &nbsp; &nbsp; </span></font>bonnes reponses</p>",
                                    font: ['MagistralC', [49, "px"], "rgba(0,0,0,1)", "600", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'nombre',
                                    type: 'text',
                                    rect: ['250px', '39px', '222px', '75px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 800; font-size: 87px; color: rgb(193, 128, 2);\">0</span></p>",
                                    align: "left",
                                    font: ['MagistralC', [91, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['500px', '309px', '650px', '75px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            align: "left",
                            font: ['MagistralC', [55, "px"], "rgba(239,0,0,1.00)", "600", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1076px', 'auto', 'auto'],
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
                            "eid300",
                            "width",
                            0,
                            0,
                            "linear",
                            "${intergace}",
                            '1450px',
                            '1450px'
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
                            rect: ['-62px', '0px', '655px', '57px', 'auto', 'auto'],
                            align: 'center',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            textStyle: ['', '', '', '', 'none']
                        },
                        {
                            type: 'text',
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Quiz</span></p>',
                            rect: ['208px', '-1px', '699px', '57px', 'auto', 'auto'],
                            align: 'center',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
                            textStyle: ['', '', '', '', 'none']
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0100_QuizDomaine_edgeActions.js");
})("EDGE-6689876");
