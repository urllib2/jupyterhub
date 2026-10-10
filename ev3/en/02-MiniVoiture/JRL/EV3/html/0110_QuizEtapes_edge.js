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
                            id: 'CASE_DEFINITION_BLUE',
                            type: 'image',
                            rect: ['416px', '132px', '770px', '135px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_ORANGE1.png",'0px','0px']
                        },
                        {
                            id: 'Definition',
                            type: 'text',
                            rect: ['452px', '173px', '959px', '112px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 36px; font-weight: 400;\">Order the steps of creating a robot.</span></p><p style=\"margin:0px\"><span style=\"font-weight: 400; font-size: 36px;\">​</span></p>",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'resultat',
                            type: 'group',
                            rect: ['366px', '277px', '1025', '111', 'auto', 'auto'],
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
                            id: 'zones',
                            type: 'group',
                            rect: ['275', '496px', '1077', '242px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'sep',
                                type: 'image',
                                rect: ['773px', '2px', '57px', '238px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'sepCopy',
                                type: 'image',
                                rect: ['247px', '0px', '57px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'sepCopy2',
                                type: 'image',
                                rect: ['510px', '1px', '57px', '239px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'sepCopy3',
                                type: 'image',
                                rect: ['773px', '2px', '57px', '238px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"sep.png",'0px','0px']
                            },
                            {
                                id: 'pack',
                                type: 'image',
                                rect: ['0px', '0px', '1077px', '240px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pack.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'ROBOT07',
                            type: 'image',
                            rect: ['282px', '110px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                        },
                        {
                            id: 'droppables',
                            type: 'group',
                            rect: ['290px', '569px', '1018', '207', 'auto', 'auto'],
                            c: [
                            {
                                id: 'basket_1',
                                type: 'rect',
                                rect: ['22px', '8px', '158px', '98px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"],
                                transform: [[],[],[],['0.71']]
                            },
                            {
                                id: 'basket_4',
                                type: 'rect',
                                rect: ['844px', '3px', '146px', '89px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'basket_2',
                                type: 'rect',
                                rect: ['288px', '3px', '162px', '98px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"],
                                transform: [[],[],[],['0.69']]
                            },
                            {
                                id: 'basket_3',
                                type: 'rect',
                                rect: ['535px', '0px', '191px', '98px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [0,"rgba(0,0,0,1)","none"],
                                transform: [[],[],[],['0.68']]
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['363px', '833px', '1040', '82', 'auto', 'auto'],
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
                                rect: ['909px', '18px', '132px', '50px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Validate</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'drop_3',
                            type: 'image',
                            rect: ['300px', '300px', '240px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A2_Programmation.png",'0px','0px']
                        },
                        {
                            id: 'drop_4',
                            type: 'image',
                            rect: ['830px', '300px', '241px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A2_Test.png",'0px','0px']
                        },
                        {
                            id: 'drop_1',
                            type: 'image',
                            rect: ['563px', '300px', '247px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A2_Conception.png",'0px','0px']
                        },
                        {
                            id: 'drop_2',
                            type: 'image',
                            rect: ['1100px', '300px', '222px', '165px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A2_Construction.png",'0px','0px']
                        },
                        {
                            id: 'Text',
                            type: 'text',
                            rect: ['270px', '813px', '594px', '75px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-family: MagistralC;\">​</span><span style=\"font-family: MagistralC; font-weight: 600;\">​</span><span style=\"font-family: MagistralC; font-weight: 600; font-size: 50px;\">​</span><span style=\"font-family: MagistralC; font-weight: 600; font-size: 50px; color: rgb(255, 0, 0);\">​</span></p>",
                            font: ['MagistralC', [50, "px"], "rgba(255,0,0,1.00)", "600", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'reset',
                            type: 'group',
                            rect: ['211px', '833px', '1040', '82', 'auto', 'auto'],
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
                                id: 'Text3',
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
                            rect: ['1223px', '110px', '232', '73', 'auto', 'auto'],
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
                            rect: ['1230px', '110px', '222px', '73', 'auto', 'auto'],
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
                            id: 'Rectangle',
                            type: 'rect',
                            rect: ['1734px', '86px', '4px', '2px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Text3Copy2',
                            type: 'text',
                            rect: ['334px', '736px', '204px', '57px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"color: rgb(193, 128, 2); font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; font-weight: 700; font-size: 40px;\">Ideation</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text3Copy3',
                            type: 'text',
                            rect: ['609px', '738px', '255px', '57px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; color: rgb(193, 128, 2); font-weight: 700; font-size: 40px;\">Design</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text3Copy4',
                            type: 'text',
                            rect: ['758px', '737px', '384px', '60px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; text-align: center; color: rgb(193, 128, 2); font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif;\">​<span style=\"font-weight: 700; font-size: 40px;\">Prototyping</span></p><p style=\"margin: 0px; text-align: center; color: rgb(193, 128, 2); font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif;\"><span style=\"font-weight: 700; font-size: 40px;\"></span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text3Copy5',
                            type: 'text',
                            rect: ['1037px', '739px', '402px', '57px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; text-align: center;\">​<span style=\"font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; color: rgb(193, 128, 2); font-weight: 700; font-size: 40px;\">Programming</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'vertrouge',
                            type: 'group',
                            rect: ['296', '508px', '1036', '207px', 'auto', 'auto'],
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
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['536px', '29', '390', '57', 'auto', 'auto']
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0110_QuizEtapes_edgeActions.js");
})("EDGE-6689876");
