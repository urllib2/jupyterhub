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
                            id: 'progL2',
                            type: 'image',
                            rect: ['312px', '304px', '903px', '178px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A2_progL2.jpg",'0px','0px']
                        },
                        {
                            id: 'wait2s2',
                            type: 'image',
                            rect: ['731px', '309px', '210px', '178px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A2_wait2s.png",'0px','0px']
                        },
                        {
                            id: 'definiktion',
                            type: 'group',
                            rect: ['363px', '106px', '824', '140', 'auto', 'auto'],
                            c: [
                            {
                                id: 'case',
                                symbolName: 'case',
                                type: 'rect',
                                rect: ['367px', '196px', '210', '183', 'auto', 'auto']
                            },
                            {
                                id: 'CASE_DEFINITION_BLUE2',
                                type: 'image',
                                rect: ['74px', '22px', '772px', '130px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['117px', '42px', '719px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Le bloc attendre permet d'arreter le&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​programme pendant un temps bien définis</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [35, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['-73px', '7px', '161px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Gmotor',
                            symbolName: 'Gmotor_1',
                            type: 'rect',
                            rect: ['442px', '265px', '61', '87', 'auto', 'auto'],
                            cursor: 'pointer'
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
                    duration: 7750,
                    autoPlay: false,
                    data: [
                            [ "eid234", "trigger", 0, function executeSymbolFunction(e, data) { this._executeSymbolAction(e, data); }, ['play', '${Gmotor}', [] ] ]
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
                            filter: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            fi: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            transform: [[], [], [], ['0.4', '0.4']],
                            fill: ['rgba(93,216,223,1.00)']
                        },
                        {
                            type: 'image',
                            fi: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            filter: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            id: 'attention-01',
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
                            filter: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            fi: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            transform: [[], [], [], ['0.4', '0.4']],
                            fill: ['rgba(93,216,223,1.00)']
                        },
                        {
                            type: 'image',
                            fi: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            filter: [0, 0, 1, 1, 0, 0, 0, 0, 'rgba(0,0,0,0)', 0, 0, 0],
                            id: 'attention-01',
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
            "case": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            rect: ['0px', '0px', '186px', '159px', 'auto', 'auto'],
                            borderRadius: ['10px', '10px', '10px', '10px'],
                            id: 'RoundRect',
                            stroke: [12, 'rgba(255,0,0,1.00)', 'solid'],
                            type: 'rect',
                            fill: ['rgba(192,192,192,0.00)']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '210px', '183px']
                        }
                    }
                },
                timeline: {
                    duration: 2000,
                    autoPlay: true,
                    data: [
                        [
                            "eid222",
                            "scaleX",
                            0,
                            1000,
                            "linear",
                            "${RoundRect}",
                            '1',
                            '1.1'
                        ],
                        [
                            "eid224",
                            "scaleX",
                            1000,
                            1000,
                            "linear",
                            "${RoundRect}",
                            '1.1',
                            '1'
                        ],
                        [
                            "eid230",
                            "scaleY",
                            0,
                            1000,
                            "linear",
                            "${RoundRect}",
                            '1',
                            '1.1'
                        ],
                        [
                            "eid232",
                            "scaleY",
                            1000,
                            1000,
                            "linear",
                            "${RoundRect}",
                            '1.1',
                            '1'
                        ]
                    ]
                }
            },
            "Gmotor_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'image',
                            display: 'block',
                            rect: ['19px', '348px', '207px', '170px', 'auto', 'auto'],
                            id: 'grand',
                            fill: ['rgba(0,0,0,0)', 'images/A2_wait2s.png', '0px', '0px']
                        },
                        {
                            type: 'group',
                            display: 'none',
                            id: 'Text4',
                            rect: ['446px', '217px', '582', '430', 'auto', 'auto'],
                            c: [
                            {
                                transform: [[], [], [], ['0.7886']],
                                type: 'image',
                                rect: ['-559px', '110px', '307px', '179px', 'auto', 'auto'],
                                id: 'blocGmotor',
                                display: 'none',
                                clip: 'rect(0px 307px 179px 6.25px)',
                                fill: ['rgba(0,0,0,0)', 'images/A2_wait2s.png', '0px', '0px']
                            },
                            {
                                rect: ['-82px', '94px', '628px', '170px', 'auto', 'auto'],
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                fill: ['rgba(0,0,0,0)', 'images/CASE_DEFINITION_GREEN1.png', '0px', '0px']
                            },
                            {
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, 'px'], 'rgba(0,0,0,1)', '500', 'none', 'normal', 'break-word', 'normal'],
                                type: 'text',
                                align: 'center',
                                id: 'Text6',
                                textStyle: ['', '', '', '', 'none'],
                                text: '<p style=\"margin: 0px; text-align: left; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 36px;\">Le bloc attendre permet d\'arrêter le&nbsp;</span>​<span style=\"font-size: 36px;\">programme pendant un temps bien définis</span></p><p style=\"margin:0px\">​</p>',
                                rect: ['-39px', '110px', '590px', '190px', 'auto', 'auto']
                            },
                            {
                                rect: ['-469px', '252px', '31px', '32px', 'auto', 'auto'],
                                id: 'sensetive_area_1Copy',
                                symbolName: 'sensetive_area_1_2_1',
                                type: 'rect'
                            },
                            {
                                rect: ['-375px', '248px', '31px', '32px', 'auto', 'auto'],
                                id: 'sensetive_area_Etat',
                                symbolName: 'sensetive_area_1_2_2',
                                type: 'rect'
                            }]
                        },
                        {
                            transform: [[], [], [], ['0.58591', '0.58591']],
                            type: 'group',
                            id: 'Tab1_arrows_etat',
                            opacity: '0.02',
                            rect: ['46px', '406px', '505px', '291px', 'auto', 'auto'],
                            c: [
                            {
                                rect: ['86px', '16px', '343px', '188px', 'auto', 'auto'],
                                id: 'Arr1Copy',
                                type: 'group',
                                c: [
                                {
                                    rect: ['-97px', '-110px', '408px', '157px', 'auto', 'auto'],
                                    id: 'arrow_1Copy',
                                    type: 'image',
                                    fill: ['rgba(0,0,0,0)', 'images/CASE_BULLES_GAUCHE_DESSOUS.png', '0px', '0px']
                                },
                                {
                                    transform: [[], [], [], ['1.41028', '1.41028']],
                                    id: 'Arr_Text1Copy',
                                    textStyle: ['', '', '32px', '', ''],
                                    font: ['MagistralC', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    type: 'text',
                                    rect: ['-24px', '-65px', '271px', '47px', 'auto', 'auto'],
                                    align: 'center',
                                    text: '<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Durée d\'attente</span></p>'
                                }]
                            }]
                        },
                        {
                            type: 'group',
                            id: 'Tab1_arrowsCopy',
                            opacity: '0.02',
                            rect: ['-341px', '429px', '474px', '291px', 'auto', 'auto'],
                            clip: 'rect(0px 505px 189.35281372070313px 0px)',
                            transform: [[], [], [], ['0.58591', '0.58591']],
                            c: [
                            {
                                rect: ['70px', '17px', '343px', '188px', 'auto', 'auto'],
                                id: 'Arr1Copy2',
                                type: 'group',
                                c: [
                                {
                                    rect: ['-89px', '0px', '455px', '170px', 'auto', 'auto'],
                                    id: 'arrow_1Copy2',
                                    type: 'image',
                                    fill: ['rgba(0,0,0,0)', 'images/CASE_BULLES_DROITE_DESSUS.png', '0px', '0px']
                                },
                                {
                                    transform: [[], [], [], ['1.41028', '1.41028']],
                                    id: 'Arr_Text1Copy2',
                                    textStyle: ['', '', '32px', '', ''],
                                    font: ['MagistralC', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    type: 'text',
                                    rect: ['0px', '88px', '279px', '43px', 'auto', 'auto'],
                                    align: 'center',
                                    text: '<p style=\"margin:0px\">​<span style=\"font-size: 34px;\">Modes</span></p>'
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
                            "eid250",
                            "scaleX",
                            2750,
                            0,
                            "linear",
                            "${blocGmotor}",
                            '0.7886',
                            '0.7886'
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
                        ],
                        [
                            "eid249",
                            "top",
                            0,
                            2492,
                            "linear",
                            "${grand}",
                            '52px',
                            '348px'
                        ],
                        [
                            "eid174",
                            "height",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '170px',
                            '179px'
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
                            "eid258",
                            "left",
                            2750,
                            0,
                            "linear",
                            "${sensetive_area_Etat}",
                            '-375px',
                            '-375px'
                        ],
                        [
                            "eid248",
                            "left",
                            0,
                            2492,
                            "linear",
                            "${grand}",
                            '300px',
                            '19px'
                        ],
                        [
                            "eid235",
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
                            "eid256",
                            "left",
                            2750,
                            0,
                            "linear",
                            "${Tab1_arrows_etat}",
                            '46px',
                            '46px'
                        ],
                        [
                            "eid259",
                            "left",
                            2750,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '-469px',
                            '-469px'
                        ],
                        [
                            "eid255",
                            "left",
                            2750,
                            0,
                            "linear",
                            "${Text4}",
                            '446px',
                            '446px'
                        ],
                        [
                            "eid251",
                            "left",
                            2750,
                            0,
                            "linear",
                            "${blocGmotor}",
                            '-559px',
                            '-559px'
                        ],
                        [
                            "eid262",
                            "top",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '429px',
                            '429px'
                        ],
                        [
                            "eid260",
                            "left",
                            2750,
                            4500,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '-358px',
                            '-341px'
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
                            rect: ['-53px', '-3px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​le bloc attendre</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['13px', '-3px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">le bloc attendre</p>',
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0200_BlocAttendre_edgeActions.js");
})("EDGE-6689876");
