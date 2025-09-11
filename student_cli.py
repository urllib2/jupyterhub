#!/usr/bin/env python3
import sys
import argparse
from student_management import StudentManager

def main():
    manager = StudentManager()
    
    parser = argparse.ArgumentParser(description='ROSForge Student Management')
    subparsers = parser.add_subparsers(dest='command', help='Commands')
    
    # Add student
    add_parser = subparsers.add_parser('add', help='Add new student')
    add_parser.add_argument('github', help='GitHub username')
    add_parser.add_argument('first_name', help='First name')
    add_parser.add_argument('last_name', help='Last name')
    add_parser.add_argument('--payment', default='unknown', choices=['cash', 'bank', 'transfer', 'other'], help='Payment method')
    add_parser.add_argument('--amount', type=float, default=0.0, help='Payment amount')
    add_parser.add_argument('--email', help='Email address')
    add_parser.add_argument('--notes', help='Additional notes')
    
    # Remove student
    remove_parser = subparsers.add_parser('remove', help='Remove student access')
    remove_parser.add_argument('github', help='GitHub username')
    
    # List students
    list_parser = subparsers.add_parser('list', help='List students')
    list_parser.add_argument('--status', choices=['active', 'inactive', 'all'], default='all', help='Filter by status')
    
    # Statistics
    stats_parser = subparsers.add_parser('stats', help='Show statistics')
    
    args = parser.parse_args()
    
    if args.command == 'add':
        success = manager.add_student(
            args.github, args.first_name, args.last_name,
            payment_method=args.payment, payment_amount=args.amount,
            email=args.email, notes=args.notes
        )
        if success:
            print("✅ Student added successfully!")
            print("Remember to send welcome email to the student.")
        else:
            print("❌ Failed to add student")
            sys.exit(1)
            
    elif args.command == 'remove':
        success = manager.remove_student(args.github)
        if success:
            print("✅ Student access removed!")
        else:
            print("❌ Student not found")
            sys.exit(1)
            
    elif args.command == 'list':
        students = manager.list_students(args.status)
        if students:
            print(f"📋 {args.status.title()} Students:")
            print(f"{'GitHub':<20} {'Name':<25} {'Enrolled':<12} {'Payment':<10} {'Amount':<8}")
            print("=" * 85)
            for student in students:
                name = f"{student['first_name']} {student['last_name']}"
                print(f"{student['github_username']:<20} {name:<25} {student['enrollment_date']:<12} {student['payment_method']:<10} ${student['payment_amount']:<8}")
        else:
            print("No students found.")
            
    elif args.command == 'stats':
        stats = manager.get_statistics()
        print("📊 Cohort Statistics:")
        print(f"Total students: {stats['total_students']}")
        print(f"Active students: {stats['active_students']}")
        print(f"Recent enrollments (30d): {stats['recent_enrollments_30d']}")
        print("\nPayment Methods:")
        for method, data in stats['payment_breakdown'].items():
            print(f"  {method}: {data['count']} students, ${data['total_amount']:.2f} total")
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
