#!/bin/bash

for name in 'd1_p4_jaune_1' 'd1_p4_gris_1' 'd1_p5_jaune_1' 'd1_p5_gris_1' 'd1_p6_jaune_1' 'd1_p6_gris_1' 'd2_p7_jaune_1' 'd2_p7_gris_1' 'd3_p7_gris_1' 'd1_p4_jaune_2' 'd1_p4_gris_2' 'd1_p5_jaune_2' 'd1_p5_gris_2' 'd1_p6_jaune_2' 'd1_p6_gris_2' 'd2_p7_jaune_2' 'd2_p7_gris_2' 'd3_p7_gris_2'
do
	for pair in 'Sujet1_Aurélie&Sujet2_Stanislas' 'Sujet1_Rémy&Sujet2_Margaux' 'Sujet1_Zaki&Sujet2_Yanis' 'Sujet1_Thanh&Sujet2_Diane' 'Sujet1_Sabrina&Sujet2_Quentin' 'Sujet1_Aniss&Sujet2_Louise' 'Sujet1_Hugo&Sujet2_Alexandre' 'Sujet1_Alexia&Sujet2_Bénédicte' 'Sujet1_Adénikè&Sujet2_Médéric' 'Sujet1_Anaïs&Sujet2_Mariem' 'Sujet1_Stéphane&Sujet2_Angélique' 'Sujet1_Fanny&Sujet2_William' 'Sujet1_Romane&Sujet2_Corentin' 'Sujet1_Paul&Sujet2_Mathieu' 'Sujet1_Marine&Sujet2_Hélène' 'Sujet1_Sébastien&Sujet2_Nils' 'Sujet1_Antoine&Sujet2_Médéric_LAAS' 'Sujet1_Amaury&Sujet2_Jason' 'Sujet1_Guilhem&Sujet2_César' 'Sujet1_Alexis&Sujet2_Thibaud'
	do
		python start_data_generation.py $name $pair 75 100
	done
done