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
                            rect: ['85px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['118px', '29px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Group',
                            type: 'group',
                            rect: ['126px', '81px', '1324px', '815', 'auto', 'auto'],
                            clip: 'rect(45.1923828125px 1260.5380859375px 815px 85.5771484375px)',
                            cursor: 'default',
                            c: [
                            {
                                id: 'ev3_interface',
                                type: 'image',
                                rect: ['88px', '44px', '1164px', '771px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_ev3_interface2.PNG",'0px','0px']
                            },
                            {
                                id: 'Tab1_arrows_etat',
                                type: 'group',
                                rect: ['665px', '499px', '505px', '291px', 'auto', 'auto'],
                                clip: 'rect(0px 505px 225.35604858398438px 0px)',
                                opacity: '0.02',
                                transform: [[],[],[],['0.58591','0.58591']],
                                c: [
                                {
                                    id: 'Arr1Copy4',
                                    type: 'group',
                                    rect: ['86px', '16px', '343px', '188px', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'arrow_1Copy4',
                                        type: 'image',
                                        rect: ['-50px', '-11px', '431px', '228px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"arrow_l_up4.png",'0px','0px']
                                    },
                                    {
                                        id: 'Arr_Text1Copy4',
                                        type: 'text',
                                        rect: ['36px', '41px', '257px', '47px', 'auto', 'auto'],
                                        text: "<p style=\"margin:0px\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 27px;\">Etat &amp; Informations de la brique&nbsp;</span></p>",
                                        align: "center",
                                        font: ['MagistralC', [25, "px"], "rgba(0,0,0,1)", "700", "none", "normal", "break-word", ""],
                                        textStyle: ["", "", "32px", "", ""],
                                        transform: [[],[],[],['1.41028','1.41028']]
                                    }]
                                }]
                            },
                            {
                                id: 'Tab1_arrows_programme',
                                type: 'group',
                                rect: ['68px', '44px', '505px', '291px', 'auto', 'auto'],
                                clip: 'rect(29.5401554107666px 485.3065490722656px 291px 0px)',
                                opacity: '0.02',
                                transform: [[],[],[],['0.58591','0.58591']],
                                c: [
                                {
                                    id: 'Arr1Copy3',
                                    type: 'group',
                                    rect: ['84px', '21px', '344px', '189px', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'arrow_1Copy3',
                                        type: 'image',
                                        rect: ['0px', '1px', '378px', '189px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"arrow_r_dn22.png",'0px','0px']
                                    },
                                    {
                                        id: 'Arr_Text1Copy3',
                                        type: 'text',
                                        rect: ['74px', '83px', '243px', '42px', 'auto', 'auto'],
                                        text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0); font-size: 26px;\">Créer un nouveau&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0); font-size: 26px;\">​programme</span></p>",
                                        align: "center",
                                        font: ['MagistralC', [25, "px"], "rgba(60,57,93,1.00)", "700", "none", "normal", "break-word", ""],
                                        textStyle: ["", "", "32px", "", ""],
                                        transform: [[],[],[],['1.41028','1.41028']]
                                    }]
                                }]
                            },
                            {
                                id: 'sensetive_area_programme',
                                symbolName: 'sensetive_area_1_2',
                                type: 'rect',
                                rect: ['214px', '97px', '31px', '32px', 'auto', 'auto']
                            },
                            {
                                id: 'Tab1_arrows_projet',
                                type: 'group',
                                rect: ['149px', '23px', '505px', '291px', 'auto', 'auto'],
                                clip: 'rect(0px 505px 202.38037109375px 85.33739471435547px)',
                                opacity: '0.02',
                                transform: [[],[],[],['0.58591','0.58591']],
                                c: [
                                {
                                    id: 'Arr1Copy2',
                                    type: 'group',
                                    rect: ['84px', '21px', '344px', '189px', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'arrow_1Copy2',
                                        type: 'image',
                                        rect: ['0px', '5px', '347px', '151px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"arrow_r_dn2.png",'0px','0px']
                                    },
                                    {
                                        id: 'Arr_Text1Copy2',
                                        type: 'text',
                                        rect: ['76px', '84px', '213px', '42px', 'auto', 'auto'],
                                        text: "<p style=\"margin:0px\"><span style=\"color: rgb(0, 0, 0); font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 26px;\">Créer un projet</span></p>",
                                        align: "center",
                                        font: ['MagistralC', [25, "px"], "rgba(60,57,93,1.00)", "700", "none", "normal", "break-word", ""],
                                        textStyle: ["", "", "32px", "", ""],
                                        transform: [[],[],[],['1.41028','1.41028']]
                                    }]
                                }]
                            },
                            {
                                id: 'sensetive_area_projet',
                                symbolName: 'sensetive_area_1_2',
                                type: 'rect',
                                rect: ['260px', '67px', '31px', '32px', 'auto', 'auto'],
                                cursor: 'pointer'
                            },
                            {
                                id: 'Tab1_arrowsCopy',
                                type: 'group',
                                rect: ['81px', '244px', '505px', '291px', 'auto', 'auto'],
                                clip: 'rect(29.5401554107666px 505px 291px 0px)',
                                opacity: '0.02',
                                transform: [[],[],[],['0.58591','0.58591']],
                                c: [
                                {
                                    id: 'Arr1Copy',
                                    type: 'group',
                                    rect: ['86px', '17px', '343px', '188px', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'arrow_1Copy',
                                        type: 'image',
                                        rect: ['0', '0', '343px', '188px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"arrow_r_dn2.png",'0px','0px']
                                    },
                                    {
                                        id: 'Arr_Text1Copy',
                                        type: 'text',
                                        rect: ['75px', '102px', '213px', '43px', 'auto', 'auto'],
                                        text: "<p style=\"margin:0px\"><span style=\"font-size: 27px; font-weight: 800; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Bloc de Début</span></p><p style=\"margin:0px\"><span style=\"font-size: 27px; font-weight: 800; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p>",
                                        align: "center",
                                        font: ['MagistralC', [27, "px"], "rgba(0,0,0,1)", "700", "none", "normal", "break-word", ""],
                                        textStyle: ["", "", "32px", "", ""],
                                        transform: [[],[],[],['1.41028','1.41028']]
                                    }]
                                }]
                            },
                            {
                                id: 'Tab1_arrows',
                                type: 'group',
                                rect: ['75px', '509px', '505px', '291px', 'auto', 'auto'],
                                opacity: '0.000000',
                                transform: [[],[],[],['0.58591','0.58591']],
                                c: [
                                {
                                    id: 'Arr1',
                                    type: 'group',
                                    rect: ['84px', '22px', '344px', '189px', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'arrow_1',
                                        type: 'image',
                                        rect: ['50px', '9px', '353px', '161px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"arrow_l_up4.png",'0px','0px']
                                    },
                                    {
                                        id: 'Arr_Text1',
                                        type: 'text',
                                        rect: ['114px', '54px', '213px', '42px', 'auto', 'auto'],
                                        text: "<p style=\"margin:0px\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Palette des blocs</span></p>",
                                        align: "center",
                                        font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1)", "700", "none", "normal", "break-word", ""],
                                        textStyle: ["", "", "32px", "", ""],
                                        transform: [[],[],[],['1.41028','1.46']]
                                    }]
                                }]
                            },
                            {
                                id: 'sensetive_area_1Copy',
                                symbolName: 'sensetive_area_1_2',
                                type: 'rect',
                                rect: ['461px', '707px', '31px', '32px', 'auto', 'auto'],
                                cursor: 'pointer'
                            },
                            {
                                id: 'sensetive_area_1',
                                symbolName: 'sensetive_area_1_2',
                                type: 'rect',
                                rect: ['213px', '279px', '31px', '32px', 'auto', 'auto'],
                                cursor: 'pointer'
                            },
                            {
                                id: 'sensetive_area_Etat',
                                symbolName: 'sensetive_area_1_2',
                                type: 'rect',
                                rect: ['1032px', '693px', '31px', '32px', 'auto', 'auto'],
                                cursor: 'pointer'
                            }]
                        },
                        {
                            id: 'TextCopy',
                            type: 'group',
                            rect: ['344px', '129px', '803', '169', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT08',
                                type: 'image',
                                rect: ['-29px', '-15px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['110px', '-14px', '848px', '153px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['137px', '-8px', '847px', '141px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">&nbsp;Le programme </span><span style=\"font-size: 34px; font-weight: 700;\">EV3-G</span><span style=\"font-size: 34px;\"> est une&nbsp;interface de programmation graphique permettant d'écrire des programmes en utilisant des </span><span style=\"font-size: 34px; font-weight: 700;\">par blocs</span><span style=\"font-size: 34px;\">&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 35px;\"></span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'legosoft',
                                type: 'image',
                                rect: ['-56px', '192px', '1107px', '588px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"legosoft.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['-194px', '270px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['1.09113']],
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
                                id: 'Text33',
                                type: 'text',
                                rect: ['902px', '13px', '142px', '44px', 'auto', 'auto'],
                                clip: 'rect(0px 140.86962890625px 44px 0px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 23px; font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; font-weight: 900;\"> &nbsp;D​​​écouvrir</span></p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'validerCopy2',
                            type: 'group',
                            rect: ['-201px', '510px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1072.8671875px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['1.09113']],
                            c: [
                            {
                                id: 'ORANGECopy2',
                                type: 'image',
                                rect: ['910px', '7px', '154px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy2',
                                type: 'image',
                                rect: ['910px', '7px', '154px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy2',
                                type: 'text',
                                rect: ['925px', '14px', '142px', '44px', 'auto', 'auto'],
                                clip: 'rect(0px 140.86962890625px 44px 0px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 23px; font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; font-weight: 900;\">Comment?</span></p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'interface2',
                            type: 'video',
                            tag: 'video',
                            rect: ['267px', '302px', '765px', '588px', 'auto', 'auto'],
                            source: [vid+"interface.webm"],
                            preload: 'auto'
                        },
                        {
                            id: 'Text5',
                            type: 'text',
                            rect: ['236px', '476px', '1178px', '296px', 'auto', 'auto'],
                            clip: 'rect(-68px 1178px 296px 0px)',
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 32px; color: rgb(162, 0, 0); font-weight: 700;\">La programmation est séquentielle : le robot exécute les commandes dans l’ordre dans lequel on les met (de gauche à droite)</span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 36px;\"></span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text1',
                            type: 'text',
                            rect: ['238px', '476px', '1178px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-size: 31px;\">L'interface de programmation graphique contrôle le robot par </span><span style=\"text-decoration: underline; font-size: 44px;\">des blocs&nbsp;</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(162,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text4',
                            type: 'text',
                            rect: ['384px', '476px', '1178px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; font-size: 31px;\">Tout programme doit commencer par <span style=\"text-decoration: underline; font-size: 43px;\">le bloc de début</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(162,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text3',
                            type: 'text',
                            rect: ['449px', '476px', '1178px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-style: normal; text-decoration: none; font-size: 32px; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">Chaque bloc correspond à une action précise</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(162,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'case',
                            symbolName: 'case',
                            type: 'rect',
                            rect: ['235px', '305px', '210', '183', 'auto', 'auto'],
                            transform: [[],[],[],['0.47839','0.68873']]
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
                    duration: 6500,
                    autoPlay: false,
                    data: [
                        [
                            "eid186",
                            "top",
                            0,
                            0,
                            "linear",
                            "${sensetive_area_Etat}",
                            '693px',
                            '693px'
                        ],
                        [
                            "eid176",
                            "opacity",
                            4000,
                            500,
                            "linear",
                            "${Tab1_arrows_programme}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid177",
                            "opacity",
                            4500,
                            500,
                            "linear",
                            "${Tab1_arrows_programme}",
                            '1',
                            '0.000000'
                        ],
                        [
                            "eid172",
                            "opacity",
                            1250,
                            500,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid173",
                            "opacity",
                            1750,
                            500,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '1',
                            '0.000000'
                        ],
                        [
                            "eid181",
                            "left",
                            6000,
                            0,
                            "linear",
                            "${sensetive_area_projet}",
                            '260px',
                            '260px'
                        ],
                        [
                            "eid183",
                            "left",
                            0,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '461px',
                            '461px'
                        ],
                        [
                            "eid180",
                            "top",
                            6000,
                            0,
                            "linear",
                            "${sensetive_area_projet}",
                            '67px',
                            '67px'
                        ],
                        [
                            "eid174",
                            "opacity",
                            2500,
                            500,
                            "linear",
                            "${Tab1_arrows_projet}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid175",
                            "opacity",
                            3000,
                            500,
                            "linear",
                            "${Tab1_arrows_projet}",
                            '1',
                            '0.000000'
                        ],
                        [
                            "eid182",
                            "top",
                            6000,
                            0,
                            "linear",
                            "${sensetive_area_programme}",
                            '97px',
                            '97px'
                        ],
                        [
                            "eid178",
                            "opacity",
                            5500,
                            500,
                            "linear",
                            "${Tab1_arrows_etat}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid179",
                            "opacity",
                            6000,
                            500,
                            "linear",
                            "${Tab1_arrows_etat}",
                            '1',
                            '0.000000'
                        ],
                        [
                            "eid184",
                            "top",
                            0,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '707px',
                            '707px'
                        ],
                        [
                            "eid168",
                            "opacity",
                            0,
                            500,
                            "linear",
                            "${Tab1_arrows}",
                            '0.000000',
                            '1'
                        ],
                        [
                            "eid169",
                            "opacity",
                            500,
                            500,
                            "linear",
                            "${Tab1_arrows}",
                            '1',
                            '0.000000'
                        ]
                    ]
                }
            },
            "TITRE": {
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
                            rect: ['460px', '-2px', '606px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 627.099853515625px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Interface EV3</span><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-weight: 500;\">-</span><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">G</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['460px', '-2px', '636px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 550px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">interface EV3</span><span style=\"font-size: 62px; font-weight: 500;\">-</span><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">G</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 4250,
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
                            [0,615.78955078125,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            },
            "sensetive_area_1_2": {
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
                            rect: ['-11px', '-15px', '44px', '49px', 'auto', 'auto'],
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
                            rect: ['5px', '3px', '40', '40', 'auto', 'auto'],
                            cursor: 'pointer',
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
                            '1'
                        ],
                        [
                            "eid13",
                            "scaleY",
                            500,
                            500,
                            "linear",
                            "${Ellipse}",
                            '1',
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
                            '1'
                        ],
                        [
                            "eid12",
                            "scaleX",
                            500,
                            500,
                            "linear",
                            "${Ellipse}",
                            '1',
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
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0230_Interface_edgeActions.js");
})("EDGE-6689876");
