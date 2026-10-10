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
                            id: 'CASE_DEFINITION_BLUE2',
                            type: 'image',
                            rect: ['412px', '154px', '830px', '119px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                        },
                        {
                            id: 'Rectangle2Copy5',
                            type: 'rect',
                            rect: ['1074px', '309px', '143px', '124px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy4',
                            type: 'rect',
                            rect: ['889px', '310px', '163px', '124px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Text',
                            type: 'text',
                            rect: ['463px', '190px', '767px', '65px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">Choisir le bloc <span style=\"font-weight: 700;\">boucle </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">parmi les blocs suivants</span>:</p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Rectangle2',
                            type: 'rect',
                            rect: ['350px', '310px', '143px', '124px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1.00)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy',
                            type: 'rect',
                            rect: ['518px', '310px', '143px', '124px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['583px', '458px', '441px', '44px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 40px;\">Cocher une reponse !</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [37, "px"], "rgba(96,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Rectangle2Copy3',
                            type: 'rect',
                            rect: ['685px', '309px', '183px', '124px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'blo1',
                            type: 'image',
                            rect: ['710px', '309px', '137px', '124px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A3_boucle.png",'0px','0px']
                        },
                        {
                            id: 'blo2',
                            type: 'image',
                            rect: ['537px', '318px', '105px', '108px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A3_blo6.png",'0px','0px']
                        },
                        {
                            id: 'blo4',
                            type: 'image',
                            rect: ['904px', '304px', '124px', '129px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A3_selector.png",'0px','0px']
                        },
                        {
                            id: 'blo5',
                            type: 'image',
                            rect: ['1076px', '292px', '143px', '153px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A3_interrupt.png",'0px','0px']
                        },
                        {
                            id: 'blo6',
                            type: 'image',
                            rect: ['363px', '318px', '105px', '106px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A3_blo5.png",'0px','0px']
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['351px', '345px', '1040', '82', 'auto', 'auto'],
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
                            id: 'Gmotor',
                            symbolName: 'Gmotor',
                            type: 'rect',
                            rect: ['10px', '6px', '61', '87', 'auto', 'auto'],
                            cursor: 'pointer'
                        },
                        {
                            id: 'ROBOT06',
                            type: 'image',
                            rect: ['256px', '132px', '150px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
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
                            type: 'image',
                            display: 'block',
                            rect: ['222px', '308px', '251px', '188px', 'auto', 'auto'],
                            id: 'grand',
                            fill: ['rgba(0,0,0,0)', 'images/A3_boucle.png', '0px', '0px']
                        },
                        {
                            type: 'image',
                            display: 'none',
                            rect: ['167px', '509px', '455px', '351px', 'auto', 'auto'],
                            id: 'blocGmotor',
                            fill: ['rgba(0,0,0,0)', 'images/A3_BOUCLE123.jpg', '0px', '0px']
                        },
                        {
                            type: 'group',
                            display: 'none',
                            id: 'Text4',
                            rect: ['740px', '431px', '664px', '435px', 'auto', 'auto'],
                            c: [
                            {
                                rect: ['0px', '128px', '691px', '231px', 'auto', 'auto'],
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                fill: ['rgba(0,0,0,0)', 'images/CASE_DEFINITION_GREEN1.png', '0px', '0px']
                            },
                            {
                                rect: ['38px', '158px', '616px', '190px', 'auto', 'auto'],
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, 'px'], 'rgba(0,0,0,1)', '500', 'none', 'normal', 'break-word', 'normal'],
                                align: 'center',
                                id: 'Text6',
                                text: '<p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">Le bloc boucle ​répète la séquence &nbsp; de blocs qu\'il contient</p><p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​Quand la boucle s\'arrête, le programme continue avec les blocs qui la suivent.</p>',
                                textStyle: ['', '', '', '', 'none'],
                                type: 'text'
                            },
                            {
                                rect: ['-353px', '107px', '31px', '32px', 'auto', 'auto'],
                                id: 'sensetive_area_1Copy',
                                symbolName: 'sensetive_area_1_2_1',
                                type: 'rect'
                            },
                            {
                                rect: ['-247px', '327px', '31px', '32px', 'auto', 'auto'],
                                id: 'sensetive_area_Etat',
                                symbolName: 'sensetive_area_1_2_2',
                                type: 'rect'
                            }]
                        },
                        {
                            rect: ['437px', '718px', '505px', '217px', 'auto', 'auto'],
                            transform: [[], [], [], ['0.58591', '0.58591']],
                            id: 'Tab1_arrows_etat',
                            opacity: '0.02',
                            type: 'group',
                            c: [
                            {
                                rect: ['70px', '13px', '326px', '188px', 'auto', 'auto'],
                                id: 'Arr1Copy',
                                type: 'group',
                                c: [
                                {
                                    rect: ['-105px', '8px', '539px', '180px', 'auto', 'auto'],
                                    id: 'arrow_1Copy',
                                    type: 'image',
                                    fill: ['rgba(0,0,0,0)', 'images/arrow_r_dn2.png', '0px', '0px']
                                },
                                {
                                    transform: [[], [], [], ['1.41028', '1.41028']],
                                    id: 'Arr_Text1Copy',
                                    text: '<p style=\"margin: 0px;\">​<span style=\"font-size: 34px;\">Condition d\'arret</span></p>',
                                    font: ['MagistralC', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    type: 'text',
                                    rect: ['-34px', '106px', '413px', '47px', 'auto', 'auto'],
                                    align: 'center',
                                    textStyle: ['', '', '32px', '', '']
                                }]
                            }]
                        },
                        {
                            type: 'group',
                            id: 'Tab1_arrowsCopy',
                            opacity: '0.02',
                            rect: ['-18px', '358px', '474px', '291px', 'auto', 'auto'],
                            clip: 'rect(0px 505px 263.053466796875px 0px)',
                            transform: [[], [], [], ['0.58591', '0.58591']],
                            c: [
                            {
                                rect: ['140px', '21px', '343px', '188px', 'auto', 'auto'],
                                id: 'Arr1Copy2',
                                type: 'group',
                                c: [
                                {
                                    rect: ['0px', '1px', '367px', '249px', 'auto', 'auto'],
                                    id: 'arrow_1Copy2',
                                    type: 'image',
                                    fill: ['rgba(0,0,0,0)', 'images/arrow_l_up4.png', '0px', '0px']
                                },
                                {
                                    transform: [[], [], [], ['1.41028', '1.41028']],
                                    id: 'Arr_Text1Copy2',
                                    text: '<p style=\"margin:0px\">​<span style=\"font-size: 34px; font-family: MagistralC;\">Numéro de</span></p><p style=\"margin:0px\"><span style=\"font-size: 34px; font-family: MagistralC;\">​la boucle</span></p>',
                                    font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    type: 'text',
                                    rect: ['40px', '70px', '279px', '114px', 'auto', 'auto'],
                                    align: 'center',
                                    textStyle: ['', '', '32px', '', '']
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
                            "eid230",
                            "width",
                            7750,
                            0,
                            "linear",
                            "${Text4}",
                            '664px',
                            '664px'
                        ],
                        [
                            "eid223",
                            "height",
                            7750,
                            0,
                            "linear",
                            "${Text4}",
                            '435px',
                            '435px'
                        ],
                        [
                            "eid206",
                            "display",
                            2500,
                            0,
                            "linear",
                            "${grand}",
                            'block',
                            'none'
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
                            '657px',
                            '222px'
                        ],
                        [
                            "eid174",
                            "height",
                            0,
                            2277,
                            "linear",
                            "${grand}",
                            '110px',
                            '188px'
                        ],
                        [
                            "eid201",
                            "height",
                            2500,
                            0,
                            "linear",
                            "${grand}",
                            '188px',
                            '188px'
                        ],
                        [
                            "eid205",
                            "top",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '308px',
                            '589px'
                        ],
                        [
                            "eid212",
                            "top",
                            7250,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '107px',
                            '107px'
                        ],
                        [
                            "eid215",
                            "left",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '-18px',
                            '-18px'
                        ],
                        [
                            "eid229",
                            "left",
                            7750,
                            0,
                            "linear",
                            "${Text4}",
                            '740px',
                            '740px'
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
                            "eid209",
                            "clip",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            [0,505,263.053466796875,0],
                            [0,505,263.053466796875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid175",
                            "width",
                            0,
                            2277,
                            "linear",
                            "${grand}",
                            '163px',
                            '251px'
                        ],
                        [
                            "eid200",
                            "width",
                            2500,
                            0,
                            "linear",
                            "${grand}",
                            '251px',
                            '251px'
                        ],
                        [
                            "eid213",
                            "left",
                            7250,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '-353px',
                            '-353px'
                        ],
                        [
                            "eid216",
                            "top",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '358px',
                            '358px'
                        ],
                        [
                            "eid224",
                            "top",
                            7750,
                            0,
                            "linear",
                            "${Text4}",
                            '431px',
                            '431px'
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
                            filter: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            fi: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            transform: [[], [], [], ['0.4', '0.4']],
                            fill: ['rgba(35,167,169,1.00)']
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
                            "eid227",
                            "background-color",
                            500,
                            0,
                            "linear",
                            "${Ellipse}",
                            'rgba(35,167,169,1.00)',
                            'rgba(35,167,169,1.00)'
                        ],
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
                            fill: ['rgba(35,167,169,1.00)']
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
                            "eid228",
                            "background-color",
                            500,
                            0,
                            "linear",
                            "${Ellipse}",
                            'rgba(35,167,169,1.00)',
                            'rgba(35,167,169,1.00)'
                        ],
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
                            rect: ['-61px', '-3px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​le bloc boucle</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['41px', '-3px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Le bloc boucle</p>',
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
                            '41px',
                            '41px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0200_BlocBoucle_edgeActions.js");
})("EDGE-6689876");
