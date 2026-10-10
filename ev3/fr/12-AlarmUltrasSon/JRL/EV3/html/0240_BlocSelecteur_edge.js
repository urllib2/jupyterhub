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
                            rect: ['90px', '6px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1_1',
                            type: 'rect',
                            rect: ['573px', '36px', '390', '57', 'auto', 'auto'],
                            clip: 'rect(-13.762939453125px 535.2841796875px 67.769287109375px -67.3916015625px)'
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
                            id: 'Rectangle2Copy5',
                            type: 'rect',
                            rect: ['1059px', '307px', '150px', '130px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy3',
                            type: 'rect',
                            rect: ['871px', '307px', '150px', '130px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy4',
                            type: 'rect',
                            rect: ['687px', '307px', '150px', '130px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2Copy',
                            type: 'rect',
                            rect: ['482px', '307px', '150px', '130px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'Rectangle2',
                            type: 'rect',
                            rect: ['299px', '307px', '150px', '130px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(192,192,192,1.00)"],
                            stroke: [0,"rgb(0, 0, 0)","none"]
                        },
                        {
                            id: 'blo5',
                            type: 'image',
                            rect: ['1059px', '306px', '150px', '130px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_interrupt.png",'0px','0px']
                        },
                        {
                            id: 'blo4',
                            type: 'image',
                            rect: ['871px', '307px', '150px', '129px', 'auto', 'auto'],
                            clip: 'rect(-2px 150px 129px -2px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_selector.png",'0px','0px']
                        },
                        {
                            id: 'blo1',
                            type: 'image',
                            rect: ['690px', '307px', '143px', '129px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_boucle.png",'0px','0px']
                        },
                        {
                            id: 'blo2',
                            type: 'image',
                            rect: ['492px', '311px', '129px', '121px', 'auto', 'auto'],
                            clip: 'rect(-12.453265190124512px 145.6842041015625px 136.7554473876953px -15.684209823608398px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blo6.png",'0px','0px'],
                            transform: [[],[],[],['0.91096','0.86874']]
                        },
                        {
                            id: 'blo6',
                            type: 'image',
                            rect: ['310px', '312px', '129px', '119px', 'auto', 'auto'],
                            clip: 'rect(-16.486486434936523px 151.1351318359375px 134px -13px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"mm_blo5.png",'0px','0px'],
                            transform: [[],[],[],['0.86047','0.86047']]
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['351px', '345px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1047px 82px 900px)',
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
                            id: 'Intro',
                            type: 'group',
                            rect: ['293', '142', '949', '150', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE2',
                                type: 'image',
                                rect: ['119px', '7px', '863px', '129px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['177px', '43px', '780px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Choisir le bloc <span style=\"font-weight: 700;\">sélecteur&nbsp;</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">parmi les blocs suivants</span>:</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['-37px', '-6px', '150px', '150px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
                            }]
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
                            rect: ['848px', '308px', '251px', '188px', 'auto', 'auto'],
                            id: 'grand',
                            fill: ['rgba(0,0,0,0)', 'images/mm_selector.png', '0px', '0px']
                        },
                        {
                            type: 'image',
                            display: 'none',
                            rect: ['295px', '595px', '487px', '403px', 'auto', 'auto'],
                            id: 'blocGmotor',
                            fill: ['rgba(0,0,0,0)', 'images/mm_selecteur.jpg', '0px', '0px']
                        },
                        {
                            type: 'group',
                            display: 'none',
                            id: 'Text4',
                            rect: ['731px', '436px', '582', '430', 'auto', 'auto'],
                            c: [
                            {
                                rect: ['43px', '135px', '690px', '238px', 'auto', 'auto'],
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
                                text: '<p style=\"margin:0px\">​</p><p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">\t</p><p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-weight: 400; font-size: 26px;\">Le bloc Sélecteur est un conteneur pouvant accueillir plusieurs séquences de blocs de programmation. Chaque séquence est appelée une Condition. Un test au début du bloc Sélecteur détermine la Condition qui va être exécutée.</span></p>',
                                rect: ['71px', '105px', '633px', '190px', 'auto', 'auto']
                            },
                            {
                                rect: ['-361px', '127px', '31px', '32px', 'auto', 'auto'],
                                id: 'sensetive_area_1Copy',
                                symbolName: 'sensetive_area_1_2_1',
                                type: 'rect'
                            },
                            {
                                rect: ['-299px', '319px', '31px', '32px', 'auto', 'auto'],
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
                            rect: ['374px', '726px', '505px', '217px', 'auto', 'auto'],
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
                                    textStyle: ['', '', '32px', '', ''],
                                    font: ['MagistralC', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    type: 'text',
                                    rect: ['-26px', '86px', '413px', '47px', 'auto', 'auto'],
                                    align: 'center',
                                    text: '<p style=\"margin: 0px;\">​<span style=\"font-size: 32px;\">instruction(s) quand la condition est fausse</span></p>'
                                }]
                            }]
                        },
                        {
                            type: 'group',
                            id: 'Tab1_arrowsCopy',
                            opacity: '0.02',
                            rect: ['296px', '524px', '474px', '291px', 'auto', 'auto'],
                            clip: 'rect(0px 607.7343139648438px 263.053466796875px 0px)',
                            transform: [[], [], [], ['0.62423', '0.58591']],
                            c: [
                            {
                                rect: ['140px', '21px', '343px', '188px', 'auto', 'auto'],
                                id: 'Arr1Copy2',
                                type: 'group',
                                c: [
                                {
                                    transform: [[], ['180'], [0, 0, 0], [1, 1, 1]],
                                    id: 'arrow_1Copy2',
                                    type: 'image',
                                    rect: ['-112px', '4px', '561px', '220px', 'auto', 'auto'],
                                    fill: ['rgba(0,0,0,0)', 'images/arrow_l_up4.png', '0px', '0px']
                                },
                                {
                                    type: 'text',
                                    id: 'Arr_Text1Copy2',
                                    text: '<p style=\"margin:0px\">​<span style=\"font-size: 32px; font-family: MagistralC;\">instruction(s) quand la condition est vrai&nbsp;</span></p>',
                                    rect: ['19px', '88px', '286px', '114px', 'auto', 'auto'],
                                    align: 'center',
                                    transform: [[], [], [], ['1.28239', '1.28239']],
                                    font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [25, 'px'], 'rgba(56,51,83,1.00)', '700', 'none', 'normal', 'break-word', ''],
                                    clip: 'rect(0px 313.1634826660156px 114px 0px)',
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
                            "eid173",
                            "left",
                            0,
                            2500,
                            "linear",
                            "${grand}",
                            '848px',
                            '222px'
                        ],
                        [
                            "eid237",
                            "scaleY",
                            2500,
                            0,
                            "linear",
                            "${blocGmotor}",
                            '1',
                            '1'
                        ],
                        [
                            "eid243",
                            "scaleY",
                            3000,
                            0,
                            "linear",
                            "${blocGmotor}",
                            '1',
                            '1'
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
                            "eid262",
                            "top",
                            7250,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '127px',
                            '127px'
                        ],
                        [
                            "eid238",
                            "scaleX",
                            2500,
                            0,
                            "linear",
                            "${blocGmotor}",
                            '1',
                            '1'
                        ],
                        [
                            "eid244",
                            "scaleX",
                            3000,
                            0,
                            "linear",
                            "${blocGmotor}",
                            '1',
                            '1'
                        ],
                        [
                            "eid249",
                            "height",
                            2500,
                            500,
                            "linear",
                            "${blocGmotor}",
                            '160px',
                            '403px'
                        ],
                        [
                            "eid251",
                            "top",
                            2500,
                            500,
                            "linear",
                            "${blocGmotor}",
                            '595px',
                            '469px'
                        ],
                        [
                            "eid252",
                            "left",
                            2500,
                            500,
                            "linear",
                            "${blocGmotor}",
                            '295px',
                            '162px'
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
                            "eid271",
                            "clip",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            [0,607.7343139648438,263.053466796875,0],
                            [0,607.7343139648438,263.053466796875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
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
                            "eid250",
                            "width",
                            2500,
                            500,
                            "linear",
                            "${blocGmotor}",
                            '211px',
                            '487px'
                        ],
                        [
                            "eid264",
                            "left",
                            7250,
                            0,
                            "linear",
                            "${sensetive_area_1Copy}",
                            '-361px',
                            '-361px'
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
                            "eid266",
                            "top",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '524px',
                            '524px'
                        ],
                        [
                            "eid265",
                            "left",
                            7250,
                            0,
                            "linear",
                            "${Tab1_arrowsCopy}",
                            '296px',
                            '296px'
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
                            fill: ['rgba(135,226,231,1.00)']
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
                            filter: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            fi: [0, 0, 1, 1, 0, 0, 0, 6, 'rgba(0,0,0,0)', 0, 0, 0],
                            transform: [[], [], [], ['0.4', '0.4']],
                            fill: ['rgba(150,229,234,1.00)']
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
                            rect: ['-97px', '-7px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​<span style=\"font-weight: 500;\">le bloc selecteur</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-49px', '-7px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\"><span style=\"font-weight: 500;\">le bloc selecteur</span></p>',
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0240_BlocSelecteur_edgeActions.js");
})("EDGE-6689876");
