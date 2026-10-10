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
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'ROBOT03',
                            type: 'image',
                            rect: ['100px', '96px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
                        },
                        {
                            id: 'CASE_DEFINITION_GREEN',
                            type: 'image',
                            rect: ['200px', '220px', '712px', '361px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET.png",'0px','0px']
                        },
                        {
                            id: 'mappostion',
                            type: 'image',
                            rect: ['798px', '277px', '38px', '55px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"mappostion.png",'0px','0px']
                        },
                        {
                            id: 'Text5',
                            type: 'text',
                            rect: ['262px', '208px', '629px', '165px', 'auto', 'auto'],
                            text: "<p style=\"margin:0px\">​<span style=\"font-weight: 400;\">​</span></p><p style=\"margin: 0px; font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif; font-weight: 600; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif; font-weight: 400;\">Donner un algorithme pour aider la voiture &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; d'aller à sa destination &nbsp; &nbsp; &nbsp;en utilisant les blocs de direction suivant :</span></p><p style=\"margin: 0px; font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif; font-weight: 600; font-style: normal; text-decoration: none; font-size: 42px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400;\">​ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;Avancer la voiture</span></p><p style=\"margin: 0px; font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif; font-weight: 600; font-style: normal; text-decoration: none; font-size: 42px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400;\">​ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;Tourner la voiture à droite</span></p><p style=\"margin: 0px; font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif; font-weight: 600; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400;\">​ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;Tourner la voiture à gauche</span></p><p style=\"margin:0px\"><span style=\"font-weight: 400;\">​</span></p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'car',
                            type: 'image',
                            rect: ['377px', '302px', '80px', '36px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A1_carr.png",'0px','0px']
                        },
                        {
                            id: 'E_1',
                            type: 'group',
                            rect: ['299px', '774px', '80', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CARR_VERT0',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                            },
                            {
                                id: 'CARR_ROUGE0',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                            },
                            {
                                id: 'CARR_GRIS0',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'E_3',
                            type: 'group',
                            rect: ['399px', '774px', '80', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CARR_ROUGE1',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                            },
                            {
                                id: 'CARR_VERT1',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                            },
                            {
                                id: 'CARR_GRIS1',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'E2_1',
                            type: 'group',
                            rect: ['499px', '774px', '80', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CARR_ROUGE2',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                            },
                            {
                                id: 'CARR_VERT2',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                            },
                            {
                                id: 'CARR_GRIS2',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'E2_2',
                            type: 'group',
                            rect: ['599px', '774px', '80', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CARR_VERT3',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                            },
                            {
                                id: 'CARR_ROUGE3',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                            },
                            {
                                id: 'CARR_GRIS3',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'E3_1',
                            type: 'group',
                            rect: ['699px', '774px', '80', '75', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CARR_ROUGE4',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                            },
                            {
                                id: 'CARR_VERT4',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                            },
                            {
                                id: 'CARR_GRIS4',
                                type: 'image',
                                rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Avancer3_1',
                            type: 'image',
                            rect: ['501px', '687px', '60px', '55px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Avancer.png",'0px','0px'],
                            userClass: ""
                        },
                        {
                            id: 'tournergauche2_2',
                            type: 'image',
                            rect: ['301px', '687px', '60px', '55px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"tourner%20gauche2.png",'0px','0px']
                        },
                        {
                            id: 'Avancer2_1',
                            type: 'image',
                            rect: ['701px', '687px', '60px', '55px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Avancer.png",'0px','0px'],
                            userClass: ""
                        },
                        {
                            id: 'tournerdroite_3',
                            type: 'image',
                            rect: ['601px', '687px', '60px', '55px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"tourner%20droite2.png",'0px','0px']
                        },
                        {
                            id: 'Avancer_1',
                            type: 'image',
                            rect: ['401px', '687px', '60px', '55px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Avancer.png",'0px','0px'],
                            userClass: ""
                        },
                        {
                            id: 'resultat',
                            type: 'group',
                            rect: ['144px', '623px', '1025', '111', 'auto', 'auto'],
                            clip: 'rect(0px 976.2822265625px 110px 0px)',
                            c: [
                            {
                                id: 'message',
                                type: 'text',
                                rect: ['55px', '28px', '1025px', '83px', 'auto', 'auto'],
                                clip: 'rect(0px 872.918212890625px 83px 0px)',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 42px;\">Vous avez</span> &nbsp; &nbsp;<span style=\"font-size: 44px;\"> &nbsp; &nbsp; </span>&nbsp; &nbsp; &nbsp;<span style=\"font-size: 42px;\">bonnes reponses</span></p>",
                                font: ['MagistralC', [49, "px"], "rgba(0,0,0,1)", "600", "none", "", "break-word", "normal"],
                                transform: [[],[],[],['1.10575']]
                            },
                            {
                                id: 'nombre',
                                type: 'text',
                                rect: ['236px', '15px', '207px', '75px', 'auto', 'auto'],
                                clip: 'rect(0px 222.76953125px 95.512939453125px 0px)',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 800; font-size: 70px; color: rgb(193, 128, 2);\">0</span></p>",
                                align: "left",
                                font: ['MagistralC', [66, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'tournerdroite_1',
                            type: 'group',
                            rect: ['301px', '392px', '41', '160', 'auto', 'auto'],
                            c: [
                            {
                                id: 'tournergauche_2Copy',
                                type: 'image',
                                rect: ['0px', '125px', '40px', '35px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"tourner%20gauche2.png",'0px','0px']
                            },
                            {
                                id: 'Avancer_1Copy',
                                type: 'image',
                                rect: ['1px', '0px', '40px', '35px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Avancer.png",'0px','0px'],
                                userClass: ""
                            },
                            {
                                id: 'tournerdroite_11',
                                type: 'image',
                                rect: ['1px', '62px', '40px', '35px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"tourner%20droite2.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'MAP2',
                            type: 'image',
                            rect: ['auto', 'auto', '522px', '521px', '186px', '338px'],
                            fill: ["rgba(0,0,0,0)",im+"A1_MAP.png",'0px','0px']
                        },
                        {
                            id: 'Symbol_1',
                            symbolName: 'Symbol_1',
                            type: 'rect',
                            rect: ['976', '323', '80', '36', 'auto', 'auto']
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
                            id: 'reset',
                            type: 'group',
                            rect: ['152px', '800px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGECopy2',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy2',
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
                            id: 'solution',
                            type: 'group',
                            rect: ['311px', '800px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            c: [
                            {
                                id: 'ORANGECopy4',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy4',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy4',
                                type: 'text',
                                rect: ['908px', '17px', '145px', '35px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Solution</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['311px', '800px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGECopy3',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy3',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy3',
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
                            id: 'sol',
                            symbolName: 'sol',
                            type: 'rect',
                            rect: ['236', '775', '801', '107', 'auto', 'auto']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1_1',
                            type: 'rect',
                            rect: ['539px', '26px', '390', '57', 'auto', 'auto'],
                            transform: [[],[],[],['1.07372']]
                        },
                        {
                            id: 'Text',
                            type: 'text',
                            rect: ['200px', '583px', '689px', '73px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            font: ['MagistralC', [55, "px"], "rgba(255,0,0,1.00)", "600", "none", "", "break-word", "normal"]
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
                    duration: 6250,
                    autoPlay: false,
                    data: [

                    ]
                }
            },
            "sol": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            rect: ['-15px', '17px', '597px', '80px', 'auto', 'auto'],
                            id: 'sol_algo',
                            type: 'image',
                            display: 'none',
                            fill: ['rgba(0,0,0,0)', 'images/A1_sol_algo.png', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '801px', '107px']
                        }
                    }
                },
                timeline: {
                    duration: 3250,
                    autoPlay: false,
                    data: [
                        [
                            "eid270",
                            "display",
                            0,
                            0,
                            "linear",
                            "${sol_algo}",
                            'none',
                            'none'
                        ],
                        [
                            "eid276",
                            "display",
                            250,
                            0,
                            "linear",
                            "${sol_algo}",
                            'none',
                            'block'
                        ],
                        [
                            "eid271",
                            "display",
                            500,
                            0,
                            "linear",
                            "${sol_algo}",
                            'block',
                            'block'
                        ],
                        [
                            "eid275",
                            "top",
                            0,
                            500,
                            "linear",
                            "${sol_algo}",
                            '17px',
                            '-12px'
                        ],
                        [
                            "eid274",
                            "left",
                            0,
                            500,
                            "linear",
                            "${sol_algo}",
                            '-1038px',
                            '-819px'
                        ],
                        [
                            "eid277",
                            "left",
                            500,
                            250,
                            "linear",
                            "${sol_algo}",
                            '-819px',
                            '-760px'
                        ],
                        [
                            "eid278",
                            "left",
                            750,
                            500,
                            "linear",
                            "${sol_algo}",
                            '-760px',
                            '-594px'
                        ],
                        [
                            "eid279",
                            "left",
                            1250,
                            500,
                            "linear",
                            "${sol_algo}",
                            '-594px',
                            '-430px'
                        ],
                        [
                            "eid280",
                            "left",
                            1750,
                            500,
                            "linear",
                            "${sol_algo}",
                            '-430px',
                            '-291px'
                        ],
                        [
                            "eid281",
                            "left",
                            2250,
                            500,
                            "linear",
                            "${sol_algo}",
                            '-291px',
                            '-133px'
                        ],
                        [
                            "eid282",
                            "left",
                            2750,
                            500,
                            "linear",
                            "${sol_algo}",
                            '-133px',
                            '-15px'
                        ]
                    ]
                }
            },
            "Symbol_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            rect: ['409px', '0px', '80px', '36px', 'auto', 'auto'],
                            fi: [0, 0, 1.6, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            filter: [0, 0, 1.6, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            id: 'carMap',
                            opacity: '1',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/A1_carr.png', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '80px', '36px']
                        }
                    }
                },
                timeline: {
                    duration: 6250,
                    autoPlay: true,
                    data: [
                        [
                            "eid295",
                            "rotateZ",
                            2750,
                            352,
                            "linear",
                            "${carMap}",
                            '0deg',
                            '17deg'
                        ],
                        [
                            "eid297",
                            "rotateZ",
                            3102,
                            340,
                            "linear",
                            "${carMap}",
                            '17deg',
                            '27deg'
                        ],
                        [
                            "eid300",
                            "rotateZ",
                            3647,
                            103,
                            "linear",
                            "${carMap}",
                            '27deg',
                            '60deg'
                        ],
                        [
                            "eid291",
                            "rotateZ",
                            4000,
                            250,
                            "linear",
                            "${carMap}",
                            '60deg',
                            '90deg'
                        ],
                        [
                            "eid304",
                            "rotateZ",
                            4250,
                            750,
                            "linear",
                            "${carMap}",
                            '90deg',
                            '80deg'
                        ],
                        [
                            "eid307",
                            "rotateZ",
                            5000,
                            250,
                            "linear",
                            "${carMap}",
                            '80deg',
                            '70deg'
                        ],
                        [
                            "eid309",
                            "rotateZ",
                            5250,
                            250,
                            "linear",
                            "${carMap}",
                            '70deg',
                            '50deg'
                        ],
                        [
                            "eid311",
                            "rotateZ",
                            5500,
                            250,
                            "linear",
                            "${carMap}",
                            '50deg',
                            '30deg'
                        ],
                        [
                            "eid313",
                            "rotateZ",
                            5750,
                            250,
                            "linear",
                            "${carMap}",
                            '30deg',
                            '10deg'
                        ],
                        [
                            "eid314",
                            "rotateZ",
                            6000,
                            250,
                            "linear",
                            "${carMap}",
                            '10deg',
                            '0deg'
                        ],
                        [
                            "eid298",
                            "left",
                            2000,
                            750,
                            "linear",
                            "${carMap}",
                            '0px',
                            '292px'
                        ],
                        [
                            "eid296",
                            "left",
                            2750,
                            500,
                            "linear",
                            "${carMap}",
                            '292px',
                            '302px'
                        ],
                        [
                            "eid290",
                            "left",
                            3442,
                            205,
                            "linear",
                            "${carMap}",
                            '302px',
                            '312px'
                        ],
                        [
                            "eid301",
                            "left",
                            3647,
                            103,
                            "linear",
                            "${carMap}",
                            '312px',
                            '320px'
                        ],
                        [
                            "eid292",
                            "left",
                            4000,
                            250,
                            "linear",
                            "${carMap}",
                            '320px',
                            '327px'
                        ],
                        [
                            "eid306",
                            "left",
                            4250,
                            1000,
                            "linear",
                            "${carMap}",
                            '327px',
                            '328px'
                        ],
                        [
                            "eid312",
                            "left",
                            5250,
                            655,
                            "linear",
                            "${carMap}",
                            '328px',
                            '343px'
                        ],
                        [
                            "eid315",
                            "left",
                            5905,
                            345,
                            "linear",
                            "${carMap}",
                            '343px',
                            '409px'
                        ],
                        [
                            "eid319",
                            "filter.contrast",
                            0,
                            895,
                            "linear",
                            "${carMap}",
                            '1',
                            '1.6'
                        ],
                        [
                            "eid299",
                            "top",
                            2000,
                            750,
                            "linear",
                            "${carMap}",
                            '0px',
                            '3px'
                        ],
                        [
                            "eid294",
                            "top",
                            2750,
                            1500,
                            "linear",
                            "${carMap}",
                            '3px',
                            '19px'
                        ],
                        [
                            "eid303",
                            "top",
                            4250,
                            500,
                            "linear",
                            "${carMap}",
                            '19px',
                            '237px'
                        ],
                        [
                            "eid305",
                            "top",
                            4750,
                            500,
                            "linear",
                            "${carMap}",
                            '237px',
                            '248px'
                        ],
                        [
                            "eid308",
                            "top",
                            5250,
                            250,
                            "linear",
                            "${carMap}",
                            '248px',
                            '264px'
                        ],
                        [
                            "eid310",
                            "top",
                            5500,
                            141,
                            "linear",
                            "${carMap}",
                            '264px',
                            '277px'
                        ]
                    ]
                }
            },
            "TITRE_1_1": {
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
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Exercice</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['138px', '0px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Exercice</span></p>'
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
                        ],
                        [
                            "eid342",
                            "left",
                            5250,
                            0,
                            "linear",
                            "${Text6}",
                            '138px',
                            '138px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0140_Taxi_edgeActions.js");
})("EDGE-6689876");
