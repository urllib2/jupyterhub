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
                            rect: ['144px', '-40px', '1014', '318', 'auto', 'auto'],
                            clip: 'rect(138.70947265625px 739.71435546875px 381.594482421875px 48.494140625px)',
                            transform: [[],[],[],['1.10458','1.12352']],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['164px', '180px', '500px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Definition',
                                type: 'text',
                                rect: ['200px', '192px', '533px', '112px', 'auto', 'auto'],
                                clip: 'rect(0px 526.33349609375px 145.333251953125px 0px)',
                                text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"font-size: 34px;\">Découvrez l'histoire de la robotique ainsi que les plus surprenantes réalisations !​</span></p>",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                            }]
                        },
                        {
                            id: 'A1_HISTORIQUE',
                            type: 'image',
                            rect: ['89px', '12px', '1432px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A1_HISTORIQUE.png",'0px','0px']
                        },
                        {
                            id: 'RobotR',
                            type: 'image',
                            rect: ['111px', '142px', '180px', '185px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                        },
                        {
                            id: 'Symbol_1',
                            symbolName: 'Symbol_1',
                            type: 'rect',
                            rect: ['118px', '80px', '1384', '874', 'auto', 'auto'],
                            clip: 'rect(28.968994140625px 1384px 852.7560424804688px 0px)',
                            transform: [[],[],[],['1.01032','1.03559']]
                        },
                        {
                            id: '_1732',
                            type: 'group',
                            rect: ['136', '718', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732',
                                display: 'block',
                                type: 'image',
                                rect: ['358px', '-540px', '570px', '597px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_1732.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1',
                                type: 'rect',
                                rect: ['0px', '-22px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_1800',
                            type: 'group',
                            rect: ['372px', '642px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy',
                                display: 'block',
                                type: 'image',
                                rect: ['184px', '-474px', '570px', '597px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_1800.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy',
                                type: 'rect',
                                rect: ['-6px', '-28px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_1928',
                            type: 'group',
                            rect: ['710px', '725px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy3',
                                display: 'block',
                                type: 'image',
                                rect: ['-190px', '-579px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_1928.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy3',
                                type: 'rect',
                                rect: ['-16px', '-29px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_1950',
                            type: 'group',
                            rect: ['825px', '562px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy4',
                                display: 'block',
                                type: 'image',
                                rect: ['-329px', '-434px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_1950.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy4',
                                type: 'rect',
                                rect: ['-6px', '-22px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_1970',
                            type: 'group',
                            rect: ['1037px', '641px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy5',
                                display: 'block',
                                type: 'image',
                                rect: ['-570px', '-468px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_1970.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy5',
                                type: 'rect',
                                rect: ['-16px', '-22px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_1999',
                            type: 'group',
                            rect: ['1106px', '403px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy9',
                                display: 'block',
                                type: 'image',
                                rect: ['-612px', '-254px', '570px', '597px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_1999.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy8',
                                type: 'rect',
                                rect: ['-12px', '-19px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_2000',
                            type: 'group',
                            rect: ['1118px', '718px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy10',
                                display: 'block',
                                type: 'image',
                                rect: ['-545px', '-557px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_2000.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy9',
                                type: 'rect',
                                rect: ['-19px', '-19px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_2006',
                            type: 'group',
                            rect: ['1139px', '327px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy11',
                                display: 'block',
                                type: 'image',
                                rect: ['-664px', '-183px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_2006.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy10',
                                type: 'rect',
                                rect: ['-16px', '-29px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_2007',
                            type: 'group',
                            rect: ['1155px', '243px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy12',
                                display: 'block',
                                type: 'image',
                                rect: ['-651px', '-46px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_2007.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy11',
                                type: 'rect',
                                rect: ['-22px', '-24px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_2010',
                            type: 'group',
                            rect: ['1189px', '174px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy13',
                                display: 'block',
                                type: 'image',
                                rect: ['-669px', '23px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_2010.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy12',
                                type: 'rect',
                                rect: ['-28px', '-29px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_2012',
                            type: 'group',
                            rect: ['1201px', '562px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy14',
                                display: 'block',
                                type: 'image',
                                rect: ['-691px', '-401px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_2012.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy13',
                                type: 'rect',
                                rect: ['-20px', '-28px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: '_2015',
                            type: 'group',
                            rect: ['1212px', '483px', '249', '57', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_1732Copy15',
                                display: 'block',
                                type: 'image',
                                rect: ['-692px', '-326px', '570px', '597px', 'auto', 'auto'],
                                overflow: 'visible',
                                fill: ["rgba(0,0,0,0)",im+"A1_2016.png",'0px','0px'],
                                transform: [[],[],[],['0','0']]
                            },
                            {
                                id: 'clicable1Copy14',
                                type: 'rect',
                                rect: ['-11px', '-23px', '249px', '57px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '0',
                                fill: ["rgba(182,182,212,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['518px', '30px', '390', '57', 'auto', 'auto']
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
                    duration: 25250,
                    autoPlay: false,
                    data: [
                        [
                            "eid425",
                            "top",
                            20000,
                            1000,
                            "linear",
                            "${A1_1732Copy13}",
                            '-270px',
                            '23px'
                        ],
                        [
                            "eid380",
                            "scaleY",
                            9500,
                            1000,
                            "linear",
                            "${A1_1732Copy5}",
                            '0',
                            '1'
                        ],
                        [
                            "eid433",
                            "left",
                            22000,
                            1000,
                            "linear",
                            "${A1_1732Copy14}",
                            '-216px',
                            '-691px'
                        ],
                        [
                            "eid373",
                            "display",
                            6500,
                            0,
                            "linear",
                            "${A1_1732Copy3}",
                            'block',
                            'none'
                        ],
                        [
                            "eid383",
                            "display",
                            10659,
                            0,
                            "linear",
                            "${A1_1732Copy5}",
                            'block',
                            'none'
                        ],
                        [
                            "eid372",
                            "display",
                            3250,
                            0,
                            "linear",
                            "${A1_1732Copy}",
                            'none',
                            'block'
                        ],
                        [
                            "eid358",
                            "display",
                            4500,
                            0,
                            "linear",
                            "${A1_1732Copy}",
                            'block',
                            'none'
                        ],
                        [
                            "eid406",
                            "scaleY",
                            11000,
                            1000,
                            "linear",
                            "${A1_1732Copy9}",
                            '0',
                            '1'
                        ],
                        [
                            "eid379",
                            "top",
                            9500,
                            1000,
                            "linear",
                            "${A1_1732Copy5}",
                            '-270px',
                            '-468px'
                        ],
                        [
                            "eid378",
                            "scaleX",
                            7750,
                            1000,
                            "linear",
                            "${A1_1732Copy4}",
                            '0',
                            '1'
                        ],
                        [
                            "eid348",
                            "left",
                            1000,
                            1000,
                            "linear",
                            "${A1_1732}",
                            '-161px',
                            '358px'
                        ],
                        [
                            "eid434",
                            "scaleX",
                            22000,
                            1000,
                            "linear",
                            "${A1_1732Copy14}",
                            '0',
                            '1'
                        ],
                        [
                            "eid432",
                            "display",
                            23250,
                            0,
                            "linear",
                            "${A1_1732Copy14}",
                            'block',
                            'none'
                        ],
                        [
                            "eid407",
                            "display",
                            12250,
                            0,
                            "linear",
                            "${A1_1732Copy9}",
                            'block',
                            'none'
                        ],
                        [
                            "eid350",
                            "scaleX",
                            1000,
                            1000,
                            "linear",
                            "${A1_1732}",
                            '0',
                            '1'
                        ],
                        [
                            "eid418",
                            "left",
                            15000,
                            1000,
                            "linear",
                            "${A1_1732Copy11}",
                            '-216px',
                            '-664px'
                        ],
                        [
                            "eid409",
                            "scaleX",
                            11000,
                            1000,
                            "linear",
                            "${A1_1732Copy9}",
                            '0',
                            '1'
                        ],
                        [
                            "eid375",
                            "scaleY",
                            7750,
                            1000,
                            "linear",
                            "${A1_1732Copy4}",
                            '0',
                            '1'
                        ],
                        [
                            "eid370",
                            "scaleX",
                            5250,
                            1000,
                            "linear",
                            "${A1_1732Copy3}",
                            '0',
                            '1'
                        ],
                        [
                            "eid374",
                            "top",
                            7750,
                            1000,
                            "linear",
                            "${A1_1732Copy4}",
                            '-270px',
                            '-434px'
                        ],
                        [
                            "eid415",
                            "top",
                            15000,
                            1000,
                            "linear",
                            "${A1_1732Copy11}",
                            '-270px',
                            '-183px'
                        ],
                        [
                            "eid429",
                            "scaleX",
                            20000,
                            1000,
                            "linear",
                            "${A1_1732Copy13}",
                            '0',
                            '1'
                        ],
                        [
                            "eid353",
                            "left",
                            3250,
                            1000,
                            "linear",
                            "${A1_1732Copy}",
                            '-161px',
                            '184px'
                        ],
                        [
                            "eid431",
                            "scaleY",
                            22000,
                            1000,
                            "linear",
                            "${A1_1732Copy14}",
                            '0',
                            '1'
                        ],
                        [
                            "eid369",
                            "left",
                            5250,
                            1000,
                            "linear",
                            "${A1_1732Copy3}",
                            '-216px',
                            '-190px'
                        ],
                        [
                            "eid428",
                            "left",
                            20000,
                            1000,
                            "linear",
                            "${A1_1732Copy13}",
                            '-216px',
                            '-669px'
                        ],
                        [
                            "eid422",
                            "display",
                            19250,
                            0,
                            "linear",
                            "${A1_1732Copy12}",
                            'block',
                            'none'
                        ],
                        [
                            "eid349",
                            "top",
                            1000,
                            1000,
                            "linear",
                            "${A1_1732}",
                            '-271px',
                            '-540px'
                        ],
                        [
                            "eid430",
                            "top",
                            22000,
                            1000,
                            "linear",
                            "${A1_1732Copy14}",
                            '-270px',
                            '-401px'
                        ],
                        [
                            "eid382",
                            "left",
                            9500,
                            1000,
                            "linear",
                            "${A1_1732Copy5}",
                            '-216px',
                            '-570px'
                        ],
                        [
                            "eid381",
                            "scaleX",
                            9500,
                            1000,
                            "linear",
                            "${A1_1732Copy5}",
                            '0',
                            '1'
                        ],
                        [
                            "eid426",
                            "scaleY",
                            20000,
                            1000,
                            "linear",
                            "${A1_1732Copy13}",
                            '0',
                            '1'
                        ],
                        [
                            "eid413",
                            "left",
                            13000,
                            1000,
                            "linear",
                            "${A1_1732Copy10}",
                            '-216px',
                            '-545px'
                        ],
                        [
                            "eid351",
                            "scaleY",
                            1000,
                            1000,
                            "linear",
                            "${A1_1732}",
                            '0',
                            '1'
                        ],
                        [
                            "eid436",
                            "scaleY",
                            24000,
                            1000,
                            "linear",
                            "${A1_1732Copy15}",
                            '0',
                            '1'
                        ],
                        [
                            "eid376",
                            "display",
                            9000,
                            0,
                            "linear",
                            "${A1_1732Copy4}",
                            'block',
                            'none'
                        ],
                        [
                            "eid352",
                            "scaleX",
                            3250,
                            1000,
                            "linear",
                            "${A1_1732Copy}",
                            '0',
                            '1'
                        ],
                        [
                            "eid412",
                            "scaleX",
                            13000,
                            1000,
                            "linear",
                            "${A1_1732Copy10}",
                            '0',
                            '1'
                        ],
                        [
                            "eid405",
                            "top",
                            11000,
                            1000,
                            "linear",
                            "${A1_1732Copy9}",
                            '-271px',
                            '-254px'
                        ],
                        [
                            "eid423",
                            "left",
                            18000,
                            1000,
                            "linear",
                            "${A1_1732Copy12}",
                            '-216px',
                            '-651px'
                        ],
                        [
                            "eid416",
                            "scaleY",
                            15000,
                            1000,
                            "linear",
                            "${A1_1732Copy11}",
                            '0',
                            '1'
                        ],
                        [
                            "eid417",
                            "display",
                            16500,
                            0,
                            "linear",
                            "${A1_1732Copy11}",
                            'block',
                            'none'
                        ],
                        [
                            "eid410",
                            "top",
                            13000,
                            1000,
                            "linear",
                            "${A1_1732Copy10}",
                            '-270px',
                            '-557px'
                        ],
                        [
                            "eid419",
                            "scaleX",
                            15000,
                            1000,
                            "linear",
                            "${A1_1732Copy11}",
                            '0',
                            '1'
                        ],
                        [
                            "eid411",
                            "scaleY",
                            13000,
                            1000,
                            "linear",
                            "${A1_1732Copy10}",
                            '0',
                            '1'
                        ],
                        [
                            "eid421",
                            "scaleY",
                            18000,
                            1000,
                            "linear",
                            "${A1_1732Copy12}",
                            '0',
                            '1'
                        ],
                        [
                            "eid427",
                            "display",
                            21250,
                            0,
                            "linear",
                            "${A1_1732Copy13}",
                            'block',
                            'none'
                        ],
                        [
                            "eid366",
                            "top",
                            5250,
                            1000,
                            "linear",
                            "${A1_1732Copy3}",
                            '-270px',
                            '-579px'
                        ],
                        [
                            "eid355",
                            "top",
                            3250,
                            1000,
                            "linear",
                            "${A1_1732Copy}",
                            '-271px',
                            '-474px'
                        ],
                        [
                            "eid377",
                            "left",
                            7750,
                            1000,
                            "linear",
                            "${A1_1732Copy4}",
                            '-216px',
                            '-329px'
                        ],
                        [
                            "eid414",
                            "display",
                            14500,
                            0,
                            "linear",
                            "${A1_1732Copy10}",
                            'block',
                            'none'
                        ],
                        [
                            "eid408",
                            "left",
                            11000,
                            1000,
                            "linear",
                            "${A1_1732Copy9}",
                            '-161px',
                            '-612px'
                        ],
                        [
                            "eid357",
                            "display",
                            2250,
                            0,
                            "linear",
                            "${A1_1732}",
                            'block',
                            'none'
                        ],
                        [
                            "eid354",
                            "scaleY",
                            3250,
                            1000,
                            "linear",
                            "${A1_1732Copy}",
                            '0',
                            '1'
                        ],
                        [
                            "eid367",
                            "scaleY",
                            5250,
                            1000,
                            "linear",
                            "${A1_1732Copy3}",
                            '0',
                            '1'
                        ],
                        [
                            "eid420",
                            "top",
                            18000,
                            1000,
                            "linear",
                            "${A1_1732Copy12}",
                            '-270px',
                            '-46px'
                        ],
                        [
                            "eid438",
                            "left",
                            24000,
                            1000,
                            "linear",
                            "${A1_1732Copy15}",
                            '-216px',
                            '-692px'
                        ],
                        [
                            "eid435",
                            "top",
                            24000,
                            1000,
                            "linear",
                            "${A1_1732Copy15}",
                            '-270px',
                            '-326px'
                        ],
                        [
                            "eid439",
                            "scaleX",
                            24000,
                            1000,
                            "linear",
                            "${A1_1732Copy15}",
                            '0',
                            '1'
                        ],
                        [
                            "eid424",
                            "scaleX",
                            18000,
                            1000,
                            "linear",
                            "${A1_1732Copy12}",
                            '0',
                            '1'
                        ],
                        [
                            "eid437",
                            "display",
                            25250,
                            0,
                            "linear",
                            "${A1_1732Copy15}",
                            'block',
                            'none'
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
                            rect: ['0px', '0px', '1384px', '874px', 'auto', 'auto'],
                            stroke: [0, 'rgb(0, 0, 0)', 'none'],
                            id: 'Rectangle',
                            opacity: '0.58',
                            type: 'rect',
                            fill: ['rgba(182,182,212,1.00)']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '1384px', '874px']
                        }
                    }
                },
                timeline: {
                    duration: 1000,
                    autoPlay: true,
                    data: [
                        [
                            "eid335",
                            "opacity",
                            0,
                            1000,
                            "linear",
                            "${Rectangle}",
                            '0',
                            '0.58'
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
                            rect: ['-151px', '1px', '859px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 58px; font-family: \'Digital tech\'; font-weight: 400;\">Histoire de la robotique</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 794.799560546875px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-44px', '1px', '752px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 58px; font-family: \'Digital tech\'; font-weight: 400;\">Histoire de la robotique</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 690.54541015625px 57px -78.28271484375px)',
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
                            [0,690.54541015625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0120_Histoire_edgeActions.js");
})("EDGE-6689876");
