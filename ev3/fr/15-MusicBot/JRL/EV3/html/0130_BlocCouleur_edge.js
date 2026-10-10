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
                            id: 'Rectangle2Copy5',
                            type: 'rect',
                            rect: ['905px', '309px', '111px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy7',
                            type: 'rect',
                            rect: ['786px', '309px', '111px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy4',
                            type: 'rect',
                            rect: ['1028px', '309px', '111px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2',
                            type: 'rect',
                            rect: ['421px', '309px', '111px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1.00)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy',
                            type: 'rect',
                            rect: ['543px', '309px', '111px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['565px', '435px', '441px', '29px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">cocher une reponse !!!</p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [37, "px"], "rgba(96,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Rectangle2Copy3',
                            type: 'rect',
                            rect: ['666px', '309px', '111px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'blo1',
                            type: 'image',
                            rect: ['662px', '311px', '123px', '111px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blocGyro.png",'0px','0px']
                        },
                        {
                            id: 'blo2',
                            type: 'image',
                            rect: ['538px', '310px', '122px', '107px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blocrotation.png",'0px','0px']
                        },
                        {
                            id: 'blo3',
                            type: 'image',
                            rect: ['1031px', '311px', '111px', '107px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blocColor.png",'0px','0px']
                        },
                        {
                            id: 'blo4',
                            type: 'image',
                            rect: ['785px', '314px', '120px', '107px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm7_blocinfra.png",'0px','0px']
                        },
                        {
                            id: 'blo5',
                            type: 'image',
                            rect: ['903px', '312px', '119px', '107px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blocultra.png",'0px','0px']
                        },
                        {
                            id: 'blo6',
                            type: 'image',
                            rect: ['418px', '309px', '119px', '107px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blocTouch.png",'0px','0px']
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['278px', '324px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
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
                            id: 'Gmotor',
                            symbolName: 'Gmotor',
                            type: 'rect',
                            rect: ['442px', '265px', '61', '87', 'auto', 'auto'],
                            cursor: 'pointer'
                        },
                        {
                            id: 'head',
                            type: 'group',
                            rect: ['276px', '126px', '960', '150', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE2',
                                type: 'image',
                                rect: ['141px', '12px', '819px', '124px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['186px', '49px', '781px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Choisir le bloc<span style=\"font-weight: 700;\"> couleur</span> parmi les blocs suivants:​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['0px', '0px', '150px', '150px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['565px', '28px', '390', '57', 'auto', 'auto']
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
                    duration: 7750,
                    autoPlay: false,
                    data: [
                            [ "eid178", "trigger", 0, function executeSymbolFunction(e, data) { this._executeSymbolAction(e, data); }, ['play', '${Gmotor}', [] ] ]
                    ]
                }
            },
            "Gmotor": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            rect: ['590px', '46px', '108px', '106px', 'auto', 'auto'],
                            id: 'grand',
                            type: 'image',
                            display: 'block',
                            fill: ['rgba(0,0,0,0)', 'images/mm_blocColor.png', '0px', '0px']
                        },
                        {
                            rect: ['-181px', '286px', '242px', '188px', 'auto', 'auto'],
                            id: 'blocGmotor',
                            type: 'image',
                            display: 'none',
                            fill: ['rgba(0,0,0,0)', 'images/mm_colorsensorbloc.jpg', '0px', '0px']
                        },
                        {
                            rect: ['322px', '136px', '582', '430', 'auto', 'auto'],
                            id: 'Text4',
                            display: 'none',
                            type: 'group',
                            c: [
                            {
                                type: 'image',
                                id: 'CASE_DEFINITION_GREEN',
                                rect: ['62px', '143px', '568px', '176px', 'auto', 'auto'],
                                fill: ['rgba(0,0,0,0)', 'images/CASE_DEFINITION_GREEN.png', '0px', '0px']
                            },
                            {
                                type: 'text',
                                rect: ['89px', '83px', '533px', '190px', 'auto', 'auto'],
                                text: '<p style=\"margin:0px\">​</p><p style=\"margin:0px\">​</p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-weight: 400;\">Le mode Mesure couleur transmet le numéro de la couleur détectée à la valeur de sortie​.</span></p>',
                                id: 'Text6',
                                textStyle: ['', '', '', '', 'none'],
                                align: 'center',
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, 'px'], 'rgba(0,0,0,1)', '500', 'none', 'normal', 'break-word', 'normal']
                            },
                            {
                                type: 'rect',
                                id: 'sensetive_area_1Copy',
                                symbolName: 'sensetive_area_1_2_1',
                                rect: ['-446px', '301px', '31px', '32px', 'auto', 'auto']
                            },
                            {
                                type: 'rect',
                                id: 'sensetive_area_Etat',
                                symbolName: 'sensetive_area_1_2_2',
                                rect: ['-369px', '299px', '31px', '32px', 'auto', 'auto']
                            }]
                        },
                        {
                            type: 'group',
                            rect: ['-105px', '391px', '505px', '291px', 'auto', 'auto'],
                            id: 'Tab1_arrows_etat',
                            opacity: '0.02',
                            transform: [[], [], [], ['0.58591', '0.58591']],
                            c: [
                            {
                                type: 'group',
                                id: 'Arr1Copy',
                                rect: ['86px', '16px', '326px', '188px', 'auto', 'auto'],
                                c: [
                                {
                                    type: 'image',
                                    id: 'arrow_1Copy',
                                    rect: ['-105px', '8px', '569px', '262px', 'auto', 'auto'],
                                    fill: ['rgba(0,0,0,0)', 'images/arrow_r_dn2.png', '0px', '0px']
                                },
                                {
                                    transform: [[], [], [], ['1.41028', '1.41028']],
                                    id: 'Arr_Text1Copy',
                                    textStyle: ['', '', '32px', '', ''],
                                    rect: ['4px', '154px', '350px', '125px', 'auto', 'auto'],
                                    text: '<p style=\"margin: 0px;\">​<span style=\"font-size: 32px;\">Valeur de sortie numérique</span></p>',
                                    font: ['MagistralC', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    align: 'center',
                                    type: 'text'
                                }]
                            }]
                        },
                        {
                            type: 'group',
                            id: 'Tab1_arrowsCopy',
                            opacity: '0.02',
                            rect: ['-98px', '416px', '474px', '291px', 'auto', 'auto'],
                            clip: 'rect(0px 505px 267.9526672363281px -215.58909606933594px)',
                            transform: [[], [], [], ['0.58591', '0.74227']],
                            c: [
                            {
                                rect: ['-123px', '47px', '343px', '222px', 'auto', 'auto'],
                                id: 'Arr1Copy2',
                                clip: 'rect(0px 367.70281982421875px 221.6857147216797px -64.85637664794922px)',
                                type: 'group',
                                c: [
                                {
                                    type: 'image',
                                    id: 'arrow_1Copy2',
                                    rect: ['-40px', '-22px', '319px', '142px', 'auto', 'auto'],
                                    transform: [[], ['180'], [0, 0, 0], [1, 1, 1]],
                                    fill: ['rgba(0,0,0,0)', 'images/arrow_l_up42.png', '0px', '0px']
                                },
                                {
                                    transform: [[], [], [], ['1.41028', '1.41028']],
                                    id: 'Arr_Text1Copy2',
                                    textStyle: ['', '', '32px', '', ''],
                                    rect: ['-15px', '52px', '279px', '43px', 'auto', 'auto'],
                                    text: '<p style=\"margin:0px\"><span style=\"font-size: 32px;\">Mode​</span></p>',
                                    font: ['MagistralC', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    align: 'center',
                                    type: 'text'
                                }]
                            }]
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            overflow: 'visible',
                            rect: [null, null, '38', '39']
                        }
                    }
                },
                timeline: {
                    duration: 7750,
                    autoPlay: false,
                    data: [
                        [
                            "eid200",
                            "height",
                            6139,
                            0,
                            "linear",
                            "${CASE_DEFINITION_GREEN}",
                            '176px',
                            '176px'
                        ],
                        [
                            "eid175",
                            "width",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '108px',
                            '159px'
                        ],
                        [
                            "eid202",
                            "top",
                            6139,
                            0,
                            "linear",
                            "${CASE_DEFINITION_GREEN}",
                            '143px',
                            '143px'
                        ],
                        [
                            "eid198",
                            "display",
                            2500,
                            0,
                            "linear",
                            "${grand}",
                            'block',
                            'none'
                        ],
                        [
                            "eid186",
                            "display",
                            0,
                            0,
                            "linear",
                            "${blocGmotor}",
                            'none',
                            'none'
                        ],
                        [
                            "eid187",
                            "display",
                            2500,
                            0,
                            "linear",
                            "${blocGmotor}",
                            'none',
                            'block'
                        ],
                        [
                            "eid193",
                            "opacity",
                            6750,
                            500,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid194",
                            "opacity",
                            7250,
                            500,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '1',
                            '0.000000'
                        ],
                        [
                            "eid173",
                            "left",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '590px',
                            '-159px'
                        ],
                        [
                            "eid184",
                            "display",
                            0,
                            0,
                            "linear",
                            "${Text4}",
                            'none',
                            'none'
                        ],
                        [
                            "eid190",
                            "display",
                            395,
                            0,
                            "linear",
                            "${Text4}",
                            'none',
                            'none'
                        ],
                        [
                            "eid185",
                            "display",
                            2500,
                            0,
                            "linear",
                            "${Text4}",
                            'none',
                            'block'
                        ],
                        [
                            "eid174",
                            "height",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '106px',
                            '150px'
                        ],
                        [
                            "eid172",
                            "top",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '46px',
                            '341px'
                        ],
                        [
                            "eid204",
                            "left",
                            6139,
                            0,
                            "linear",
                            "${CASE_DEFINITION_GREEN}",
                            '62px',
                            '62px'
                        ],
                        [
                            "eid191",
                            "opacity",
                            5500,
                            500,
                            "linear",
                            "${Tab1_arrows_etat}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid192",
                            "opacity",
                            6000,
                            500,
                            "linear",
                            "${Tab1_arrows_etat}",
                            '1',
                            '0.000000'
                        ]
                    ]
                }
            },
            "sensetive_area_1_2_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: true,
                content: {
                    dom: [
                        {
                            type: 'ellipse',
                            borderRadius: ['50%', '50%', '50%', '50%'],
                            id: 'Ellipse',
                            stroke: [5, 'rgba(255,255,255,1.00)', 'solid'],
                            rect: ['0', '-1', '27', '27', 'auto', 'auto'],
                            fi: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            filter: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            transform: [[], [], [], ['0.4', '0.4']],
                            fill: ['rgba(133,225,230,1.00)']
                        },
                        {
                            type: 'image',
                            fi: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            id: 'attention-01',
                            filter: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            rect: ['4', '5', '40', '40', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/zoom.png', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '38', '39']
                        }
                    }
                },
                timeline: {
                    duration: 1000,
                    autoPlay: true,
                    data: [
                        [
                            "eid4",
                            "scaleY",
                            0,
                            500,
                            "linear",
                            "${Ellipse}",
                            '0.4',
                            '1.2'
                        ],
                        [
                            "eid13",
                            "scaleY",
                            500,
                            500,
                            "linear",
                            "${Ellipse}",
                            '1.2',
                            '0.4'
                        ],
                        [
                            "eid3",
                            "scaleX",
                            0,
                            500,
                            "linear",
                            "${Ellipse}",
                            '0.4',
                            '1.2'
                        ],
                        [
                            "eid12",
                            "scaleX",
                            500,
                            500,
                            "linear",
                            "${Ellipse}",
                            '1.2',
                            '0.4'
                        ]
                    ]
                }
            },
            "sensetive_area_1_2_2": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: true,
                content: {
                    dom: [
                        {
                            type: 'ellipse',
                            borderRadius: ['50%', '50%', '50%', '50%'],
                            id: 'Ellipse',
                            stroke: [5, 'rgba(255,255,255,1.00)', 'solid'],
                            rect: ['0', '-1', '27', '27', 'auto', 'auto'],
                            fi: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            filter: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            transform: [[], [], [], ['0.4', '0.4']],
                            fill: ['rgba(130,225,230,1.00)']
                        },
                        {
                            type: 'image',
                            fi: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            id: 'attention-01',
                            filter: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            rect: ['4', '5', '40', '40', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/zoom.png', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '38', '39']
                        }
                    }
                },
                timeline: {
                    duration: 1000,
                    autoPlay: true,
                    data: [
                        [
                            "eid4",
                            "scaleY",
                            0,
                            500,
                            "linear",
                            "${Ellipse}",
                            '0.4',
                            '1.2'
                        ],
                        [
                            "eid13",
                            "scaleY",
                            500,
                            500,
                            "linear",
                            "${Ellipse}",
                            '1.2',
                            '0.4'
                        ],
                        [
                            "eid3",
                            "scaleX",
                            0,
                            500,
                            "linear",
                            "${Ellipse}",
                            '0.4',
                            '1.2'
                        ],
                        [
                            "eid12",
                            "scaleX",
                            500,
                            500,
                            "linear",
                            "${Ellipse}",
                            '1.2',
                            '0.4'
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
                            rect: ['-106px', '0px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">&nbsp;bloc couleur</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['13px', '0px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​ bloc couleur</p>',
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
                        ],
                        [
                            "eid218",
                            "left",
                            5250,
                            0,
                            "linear",
                            "${Text6}",
                            '13px',
                            '13px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0130_BlocCouleur_edgeActions.js");
})("EDGE-6689876");
